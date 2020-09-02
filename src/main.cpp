/** 
 * WebServos
 * 
 *       Servo-->    0    1    2    3
 * PWM Output Pin   21   19   23   18 
 * PWM Channel       4    5    6    7
 * ADC Input Pin    32   33   34   35
 * 
 *  [Browser]        [ESP32]
 * currentState -->   send('state')
 * follow       -->   move Servo#
 * play         -->   noop
 * startRecord  -->   disable-move send('record') stream every interval
 * stopRecord   -->   enable-move  stop('record') stream
 * button[follow,play,startRecord,stopRecord] -> if no 'degree' send(button) only
*/ 

#include <wifiTool.h>
#include <ArduinoJson.h>          //https://github.com/bblanchon/ArduinoJson
#include <WebSocketsServer.h>

#include <Servo.h>
#include <ESP32AnalogRead.h>

#define MAX_SERVOS 4
#define MAX_REQUEST_QUEUE_SZ 32
#define MAX_RECORD_COUNT 240         // sequence, servo, degree
#define SERVO_PULSE_MIN 500
#define SERVO_PULSE_MAX 2500
#define SERVO_DEGREE_MAX 270

static const int servosPins[MAX_SERVOS] = {21, 19, 23, 18};
static const int servosPosition[MAX_SERVOS] = {32, 33, 34, 35};
static const int servoMtrChannel[MAX_SERVOS] = {0, 1, 2, 3};
static const int servoFdBKChannel[MAX_SERVOS] = {4, 5, 6, 7};
static int servoIds[MAX_SERVOS] = {0,1,2,3};
static int servoDegreeMax = 270;
static int servoRecordMax = 240;

volatile unsigned long guiTimeBase   = 0,      // default time base, target 0.5 seconds
                  gulLastTimeBase    = 0,      // time base delta
                  gulRecordInterval  = 500,    // Ticks interval 0.5 seconds
                  gulRecordCounter   = 0;      // Number of recorded records
volatile bool     gbRecordMode       = false,  // Recording
                  gvDuration         = false,  // 1/2 second marker
                  gbWSOnline = false;          // WebSocket Client Connected

QueueHandle_t qRequest[MAX_SERVOS];

volatile SemaphoreHandle_t servoMutex;

typedef struct _qitem {
  int servo;
  int degree;
  uint32_t analog;
  uint32_t current;
} QItem, *PQItem;

typedef union _CalibrationValue {
  uint32_t  value;
  unsigned char byte8[sizeof(uint32_t)];
} CalibrationValue, *PCalibrationValue;

typedef struct _servoCalibration {
  CalibrationValue current;
  CalibrationValue degree;
  CalibrationValue minPosition;
  CalibrationValue maxPosition;
  CalibrationValue timeSequence;
} ServoCalibration, *PServoCalibration;

Servo servos[MAX_SERVOS];
ESP32AnalogRead adc[MAX_SERVOS];
ServoCalibration calibrate[MAX_SERVOS];
AsyncWebServer server(80);
WebSocketsServer webSocket = WebSocketsServer(1337);
WifiTool wifiTool;

char servo_max_value[8];     // Configuration Param
char message[128];           // response messages
char recordBuffer[128];      // recording entries
char lastBuffer[128];        // filter duplicate messages

/*
 * Record generator
*/
void servoRecordRequest() {
    gulRecordCounter += 1;
    if (gulRecordCounter > MAX_RECORD_COUNT) {
      gbRecordMode = false;
      gulRecordCounter = 0;
      Serial.printf("Recording: [%lu] Limit Reached, Recording Stopped!\n", guiTimeBase);

      snprintf(recordBuffer, sizeof(recordBuffer), "{\"action\":\"follow\"}" );
      webSocket.broadcastTXT( recordBuffer, strlen(recordBuffer), false );
      return;
    }

    for(int servo = 0; servo < MAX_SERVOS; ++servo) {
      calibrate[servo].current.value = adc[servo].readMiliVolts();;
      calibrate[servo].degree.value = map( calibrate[servo].current.value, calibrate[servo].minPosition.value, calibrate[servo].maxPosition.value, 0, SERVO_DEGREE_MAX);
      if (calibrate[servo].current.value >= calibrate[servo].maxPosition.value) {
        calibrate[servo].degree.value = SERVO_DEGREE_MAX;
      } if (calibrate[servo].current.value <= calibrate[servo].minPosition.value) {
        calibrate[servo].degree.value = 0;
      } else {
        calibrate[servo].degree.value = calibrate[servo].degree.value % (SERVO_DEGREE_MAX + 1);
      }
      snprintf(recordBuffer, sizeof(recordBuffer), "{\"action\":\"record\",\"servo\":%d,\"degree\":%u}", servo, calibrate[servo].degree.value );
      Serial.printf("Recording: [%lu] %s\n", guiTimeBase, recordBuffer);
      webSocket.broadcastTXT( recordBuffer, strlen(recordBuffer), false );
    }
}

/*
 * Function to send new request
*/
void servoActionRequest(int servo, int degrees) {    
    PQItem pqi = NULL;

    pqi = (PQItem) heap_caps_malloc(sizeof( QItem ), MALLOC_CAP_8BIT);  // heap_caps_free() or free() later
    if (pqi != NULL) {
      pqi->servo  = servo;
      pqi->degree = degrees;
      xQueueSend(qRequest[servo], &pqi, portMAX_DELAY);
    } else {
      Serial.printf("Servo %d Memory Allocation Failed! pqi=%p .....................\n", servo, pqi);
    }
}

/*
 * function to re-attach servos
*/
bool attachServos() {
  bool rc = true;
  gbRecordMode = false;
  for(int i = 0; i < MAX_SERVOS; ++i) {
    if(!servos[i].attach(servosPins[i],servoMtrChannel[i], 0, SERVO_DEGREE_MAX, SERVO_PULSE_MIN, SERVO_PULSE_MAX )) {
      Serial.printf("Servo %d attach error! \n", i);
      rc = false;
    }
  }
  return rc;
}

/*
 * function to detach servos, for training
*/
bool detachServos() {
  bool rc = true;
  gbRecordMode = true;
  for(int i = 0; i < MAX_SERVOS; ++i) {
    if(!servos[i].detach()) {
      Serial.printf("Servo %d detach error! \n", i);
      rc = false;
    }
  }
  return rc;
}

// Callback: receiving any WebSocket message
void onWebSocketEvent(uint8_t client_num,
                      WStype_t type,
                      uint8_t * payload,
                      size_t length) {

  DynamicJsonDocument doc(200);
  DeserializationError errorJson;

  // Figure out the type of WebSocket event
  switch(type) {

    // Client has disconnected
    case WStype_DISCONNECTED:
      gbWSOnline = false;
      Serial.printf("[%u] Disconnected!\n", client_num);
      break;

    // New client has connected
    case WStype_CONNECTED:
      {
        IPAddress ip = webSocket.remoteIP(client_num);
        Serial.printf("[%u] Connection from ", client_num);
        Serial.println(ip.toString());
        gbWSOnline = true;
      }
      break;

    // Handle text messages from client
    case WStype_TEXT:
      if ( strcmp((char *)payload, lastBuffer) == 0 && strcmp((char *)payload, "{\"action\":\"currentState\"}") != 0) { // duplicate message
        Serial.printf("[%u] Duplicate Message Received text: %s\n", client_num, payload);
        break;
      } else {
        strncpy(lastBuffer, (char *)payload, sizeof(lastBuffer));
      }

      // Print out raw message
      Serial.printf("[%u] Received text: %s\n", client_num, payload);
 
      errorJson = deserializeJson(doc, payload);
      if(errorJson) {
        Serial.printf("deserializeJson() failed! cause=%s\n", errorJson.c_str());
        break;
      }

      // determine action
      if ( strcmp(doc["action"], "currentState") == 0 ) { // move to neutral positon -- currentState
        Serial.printf("Current State Action \n");
        for(int i = 0; i < MAX_SERVOS; ++i) {
          snprintf(message, sizeof(message), "{\"action\":\"state\",\"servo\":%d,\"degree\":%d}", i, calibrate[i].degree.value);
          webSocket.broadcastTXT( message, strlen(message), false );
          Serial.printf("Sending currentState=%s\n", message);
        }

      } else if ( strcmp(doc["action"], "follow") == 0 ) {
        Serial.printf("Follow Action \n");
        if ( !gbRecordMode && doc.containsKey("degree") ) {
          servoActionRequest(doc["servo"], doc["degree"]);        
        } else if (!doc.containsKey("degree")) {
          attachServos();
          snprintf(message, sizeof(message), "{\"action\":\"follow\"}");
          Serial.printf("Follow: %s\n", message);
          webSocket.broadcastTXT( message, strlen(message), false );
        }

      } else if ( strcmp(doc["action"], "play") == 0 ) {
        Serial.printf("Play Action \n");
        if(gbRecordMode) {
          attachServos();
        }
        snprintf(message, sizeof(message), "{\"action\":\"play\"}");
        Serial.printf("Playing: %s\n", message);
        webSocket.broadcastTXT( message, strlen(message), false );

      } else if ( strcmp(doc["action"], "startRecord") == 0 ) {
        Serial.printf("StartRecord Action \n");
        if (!gbRecordMode) {
          detachServos();
        }
        snprintf(message, sizeof(message), "{\"action\":\"startRecord\"}");
        Serial.printf("Recording: %s\n", message);
        webSocket.broadcastTXT( message, strlen(message), false );

      } else if ( strcmp(doc["action"], "stopRecord") == 0 ) {
        Serial.printf("StopRecord Action \n");
        if(gbRecordMode) {
          attachServos();
        }
        snprintf(message, sizeof(message), "{\"action\":\"stopRecord\"}");
        Serial.printf("Stop Recording: %s\n", message);
        webSocket.broadcastTXT( message, strlen(message), false );

      } else {
        Serial.println("Message not recognized!");
      }
      break;

    case WStype_ERROR:
      Serial.printf("[%u] Error Received: %s\n", client_num, payload);
      break;

    default:
      break;
  }
}

// Callback: send homepage
void onIndexRequest(AsyncWebServerRequest *request) {
  IPAddress remote_ip = request->client()->remoteIP();
  Serial.println("[" + remote_ip.toString() +
                  "] HTTP GET request of " + request->url());
  request->send(SPIFFS, "/servo.html", "text/html");
}
void onJSRequest(AsyncWebServerRequest *request) {
  IPAddress remote_ip = request->client()->remoteIP();
  Serial.println("[" + remote_ip.toString() +
                  "] HTTP GET request of " + request->url());
  request->send(SPIFFS, "/jogDial.min.js", "text/javascript");
}
void onImgKnobRequest(AsyncWebServerRequest *request) {
  IPAddress remote_ip = request->client()->remoteIP();
  Serial.println("[" + remote_ip.toString() +
                  "] HTTP GET request of " + request->url());
  request->send(SPIFFS, "/base_one_knob.png", "image/png");
}
void onImgBgRequest(AsyncWebServerRequest *request) {
  IPAddress remote_ip = request->client()->remoteIP();
  Serial.println("[" + remote_ip.toString() +
                  "] HTTP GET request of " + request->url());
  request->send(SPIFFS, "/base_one_bg.png", "image/png");
}
// Callback: send 404 if requested file does not exist
void onPageNotFound(AsyncWebServerRequest *request) {
  IPAddress remote_ip = request->client()->remoteIP();
  Serial.println("[" + remote_ip.toString() +
                  "] HTTP GET request of " + request->url());
  request->send(404, "text/plain", "Not found");
}

void calibrateServos(long degrees) {
    long servoDegrees = 0;
    
    for(int i = 0; i < MAX_SERVOS; ++i) {
      servoDegrees = degrees % SERVO_DEGREE_MAX;
      if (servoDegrees == 0 && degrees == SERVO_DEGREE_MAX ) { 
        servoDegrees = degrees;
      }
      servos[i].write(servoDegrees);      
    }

    vTaskDelay(500/portTICK_PERIOD_MS);

    for(int i = 0; i < MAX_SERVOS; ++i) {
      calibrate[i].current.value = adc[i].readMiliVolts();
      calibrate[i].minPosition.value = min(calibrate[i].minPosition.value, calibrate[i].current.value);
      calibrate[i].maxPosition.value = max(calibrate[i].maxPosition.value, calibrate[i].current.value);
      calibrate[i].degree.value = map( calibrate[i].current.value, calibrate[i].minPosition.value, calibrate[i].maxPosition.value, 0, SERVO_DEGREE_MAX);
      
      Serial.printf("Servo: %d,\t W-degrees: %ld,\t delayed-analog: %u,\t delayed-degrees: %u, min: %u, max: %u \n", 
                    i, servoDegrees, calibrate[i].current.value, calibrate[i].degree.value, calibrate[i].minPosition.value, calibrate[i].maxPosition.value);
   } 
}

void servoTask(void * pServo) {
  PQItem pqi;
  int servo  = (int)(*(int *)pServo);
  int servoDegrees = 0,
      vDelay = 0,
      lastKnownDegrees = 0;
  QueueHandle_t requestQueue = qRequest[servo];

  vTaskDelay(1000/portTICK_PERIOD_MS);

  for(;;){ // infinite loop        
    if ( xQueueReceive(requestQueue, &pqi, portMAX_DELAY) == pdTRUE) {
      if (NULL == pqi) {
        vTaskDelay(1000/portTICK_PERIOD_MS);
        continue;
      }
      servoDegrees = pqi->degree % SERVO_DEGREE_MAX;
      if (servoDegrees == 0 && pqi->degree == SERVO_DEGREE_MAX ) { 
        servoDegrees = pqi->degree;
      } 

      if( !gbRecordMode && xSemaphoreTake(servoMutex, portMUX_NO_TIMEOUT) == pdTRUE) {
        servos[servo].write(servoDegrees);  
        xSemaphoreGive(servoMutex);
        
        vDelay = (int)(abs(servoDegrees - lastKnownDegrees)/60.0 * 100.0 + 50);
        vTaskDelay(vDelay/portTICK_PERIOD_MS); // 100ms/60degrees or 450ms for SERVO_DEGREE_MAX degrees

        calibrate[servo].current.value = adc[servo].readMiliVolts();;
        calibrate[servo].degree.value = map( calibrate[servo].current.value, calibrate[servo].minPosition.value, calibrate[servo].maxPosition.value, 0, SERVO_DEGREE_MAX);
        Serial.printf("Servo %d analog: %u, degree: %u, stackHighwater: %d, QueueDepth: %d\n", 
                       servo, pqi->analog, calibrate[servo].degree.value, uxTaskGetStackHighWaterMark(NULL), uxQueueMessagesWaiting( requestQueue ));
      } 
      lastKnownDegrees = servoDegrees;
      heap_caps_free(pqi); // free allocation
    } else {
      vTaskDelay(1000/portTICK_PERIOD_MS);
    }
    pqi = NULL;
  }
  vTaskDelete( NULL );
}

void generateServoTasks() {
  static char const *pchTitle[4] = {"ServoTask-00", "ServoTask-01", "ServoTask-02", "ServoTask-03"};

  for(int servo = 0; servo < MAX_SERVOS; ++servo) {
    xTaskCreatePinnedToCore(
        servoTask,         // Function that should be called
        pchTitle[servo],   // Name of the task (for debugging)
        2048,              // Stack size (bytes)
        &servoIds[servo],  // Parameter to pass
        2,                 // Task priority
        NULL,              // Task handle
        tskNO_AFFINITY     // Run on any available core
      );
  }
}

void initializeServoControls() {
  servoMutex = xSemaphoreCreateMutex();

  for(int i = 0; i < MAX_SERVOS; ++i) {
    qRequest[i] = xQueueCreate( MAX_REQUEST_QUEUE_SZ, sizeof( PQItem ) );
    if(qRequest[i] == NULL) {
      Serial.printf("Error creating the request queue %d \n", i);
    }
      
    pinMode(servosPosition[i], INPUT);
    pinMode(servosPins[i], OUTPUT);

    adc[i].attach(servosPosition[i]);
    calibrate[i].minPosition.value = 155;
    calibrate[i].maxPosition.value = 3105;
  
    if(!servos[i].attach(servosPins[i],servoMtrChannel[i], 0, SERVO_DEGREE_MAX, SERVO_PULSE_MIN, SERVO_PULSE_MAX )) {
      Serial.printf("Servo %d attach error! \n", i);
    }
    gbRecordMode = false;
  }

  /*
    * Calibrate Servos
    * - read EEPROM first
    * - read JSON File if no EEPROM
  */
  calibrateServos(135);
  calibrateServos(0);    
  calibrateServos(135);
  calibrateServos(SERVO_DEGREE_MAX);
  calibrateServos(135);

}

JsonObject readRecordedMovements(String jsonFilePathname) {
  DynamicJsonDocument doc(4096);
  if ( SPIFFS.exists(jsonFilePathname) ) {
    //file exists, reading and loading
    Serial.printf("Reading file: %s\n", jsonFilePathname.c_str());

    File jsonFile = SPIFFS.open(jsonFilePathname, "r");
    if (jsonFile) {
      DeserializationError err = deserializeJson(doc, jsonFile);
      if (err) {
        Serial.printf("failed to load File; cause %s\n", err.c_str());
        jsonFile.close();
        return doc.to<JsonObject>();
      } 
      jsonFile.close();      
    }
  }
  return doc.as<JsonObject>();
}

bool saveRecordedMovements(const uint8_t *payload, String jsonFilePathname) {
  File jsonFile = SPIFFS.open(jsonFilePathname, "w");
  if (!jsonFile) {
    Serial.printf("Failed to open file: %s, for writing. \n", jsonFilePathname.c_str());
    return false;
  }

  jsonFile.write(payload, strlen((const char *)payload));
  jsonFile.close();

  return true;
}

void setup() {
  Serial.begin(115200);

  Serial.println("Skoona.net Multple Servos with Feedback.");

  /*
   * Init Servo
  */
  initializeServoControls();  
  /*
   * Create Servo Handler
  */
  generateServoTasks();

  if (SPIFFS.begin()) {
    Serial.println("Filesystem available!");
  } else {
    Serial.println("filesystem not available!");
  }

  wifiTool.begin(false);
  if (!wifiTool.wifiAutoConnect()) {
    Serial.println("fail to connect to wifi!!!!");
    wifiTool.runApPortal();
  }

  // On HTTP request for root, provide index.html file
  server.begin();
  server.on("/", HTTP_GET, onIndexRequest);
  server.on("/servo", HTTP_GET, onIndexRequest);
  server.on("/jogDial.min.js", HTTP_GET, onJSRequest);
  server.on("/base_one_knob.png", HTTP_GET, onImgKnobRequest);
  server.on("/base_one_bg.png", HTTP_GET, onImgBgRequest);
  server.onNotFound(onPageNotFound);

  // Start WebSocket server and assign callback
  webSocket.begin();
  webSocket.onEvent(onWebSocketEvent);
  
  Serial.printf("Setup Complete !\n");
}

void loop() {
  guiTimeBase = millis();
  gvDuration = ((guiTimeBase - gulLastTimeBase) >= gulRecordInterval);

  webSocket.loop();

  if (gbWSOnline && gvDuration && gbRecordMode) {
    servoRecordRequest();
  }

  if (gvDuration) { // Every half second poll 
    gulLastTimeBase = guiTimeBase;    
  }

  taskYIELD();
}
