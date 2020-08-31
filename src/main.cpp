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
 * startRecord  -->   disable-move send('record')
 * stopRecord   -->   enable-move  
 * 
*/ 

#include "Arduino.h"

#include <WiFi.h>
#include <SPIFFS.h>
#include <ESPAsyncWebServer.h>
#include <WebSocketsServer.h>

#include <Servo.h>
#include <ESP32AnalogRead.h>
#include <ArduinoJson.h>

#define MAX_SERVOS 4
#define MAX_REQUEST_QUEUE_SZ 32
#define MAX_RESPONSE_QUEUE_SZ 64
#define SERVO_PULSE_MIN 500
#define SERVO_PULSE_MAX 2500
#define SERVO_DEGREE_MAX 270

static int servoIds[MAX_SERVOS] = {0,1,2,3};
static const int servosPins[MAX_SERVOS] = {21, 19, 23, 18};
static const int servosPosition[MAX_SERVOS] = {32, 33, 34, 35};

static const int servoMtrChannel[MAX_SERVOS] = {0, 1, 2, 3};
static const int servoFdBKChannel[MAX_SERVOS] = {4, 5, 6, 7};

// Constants
const char* ssid = "SFNSS1-24G";
const char* password = "Apache Tomcat 8";

QueueHandle_t qRequest[MAX_SERVOS], 
                       qResponse;

volatile SemaphoreHandle_t servoMutex;

typedef struct _qitem {
  int servo;
  int degree;
  uint32_t analog;
  uint32_t current;
} QItem, *PQItem;

typedef struct _servoPosition {
  uint32_t current;
  uint32_t degree;
  uint32_t minPosition;
  uint32_t maxPosition;
} ServoPositions, *PServoPostions;


Servo servos[MAX_SERVOS];
ESP32AnalogRead adc[MAX_SERVOS];
ServoPositions calibrate[MAX_SERVOS];

AsyncWebServer server(80);
WebSocketsServer webSocket = WebSocketsServer(1337);

// StaticJsonBuffer<64> jsonBuffer;

char msg_buffer[128];
char message[128];
char lastBuffer[128];
bool online = false;
volatile int clientOnline = 0;
volatile bool servoRecordMode = false;

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
 * Task to drain response queue
*/
void servoActionResponse(void * _unused) {
  PQItem pqi = NULL;
  while ( true) {
    if( xQueueReceive(qResponse, &pqi, portMAX_DELAY) == pdTRUE ) {
      /* Do something useful with pqi */
      // '{"action":"record","servo":2,"degree":135}'
      if (online && clientOnline >= 1 && servoRecordMode ) {      
        snprintf(message, sizeof(message), "{\"action\":\"record\",\"servo\":%d,\"degree\":%d}", pqi->servo, pqi->degree);
        String mesg = String(message);
        webSocket.broadcastTXT( mesg );
      }
      heap_caps_free(pqi);
    }
  }
  vTaskDelete( NULL );
}

/*
 * function to re-attach servos
*/
bool attachServos() {
  bool rc = true;
  servoRecordMode = false;
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
  servoRecordMode = true;
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
      clientOnline -= 1;
      Serial.printf("[%u] Disconnected!\n", client_num);
      break;

    // New client has connected
    case WStype_CONNECTED:
      {
        IPAddress ip = webSocket.remoteIP(client_num);
        Serial.printf("[%u] Connection from ", client_num);
        Serial.println(ip.toString());
        clientOnline += 1;
      }
      break;

    // Handle text messages from client
    case WStype_TEXT:
      if ( strcmp((char *)payload, lastBuffer) == 0 ) { // duplicate message
        Serial.printf("[%u] Duplicate Message Received text: %s\n", client_num, payload);
        break;
      } else {
        strncpy(lastBuffer, (char *)payload, sizeof(lastBuffer));
      }

      // Print out raw message
      Serial.printf("[%u] Received text: %s\n", client_num, payload);
 
      errorJson = deserializeJson(doc, payload);
      if(errorJson) {
        Serial.printf("Invalid Input, deserializeJson() failed! cause=%s\n", errorJson.c_str());
        break;
      }

      // determine action
      if ( strcmp(doc["action"], "currentState") == 0 ) { // move to neutral positon -- currentState
        String mesg;
        Serial.printf("Current State Action \n");
        for(int i = 0; i < MAX_SERVOS; ++i) {
          snprintf(message, sizeof(message), "{\"action\":\"state\",\"servo\":%d,\"degree\":%d}", i, calibrate[i].degree);
          mesg = String(message);
          webSocket.broadcastTXT( mesg );
          Serial.printf("Sending currentState=%s\n", message);
        }

      } else if ( strcmp(doc["action"], "follow") == 0 ) {
        Serial.printf("Follow Action \n");
        if ( doc.containsKey("degree") ) {
          servoActionRequest(doc["servo"], doc["degree"]);        
        } 

      } else if ( strcmp(doc["action"], "play") == 0 ) {
        Serial.printf("Play Action \n");

      } else if ( strcmp(doc["action"], "startRecord") == 0 ) {
        Serial.printf("StartRecord Action \n");
        detachServos();

      } else if ( strcmp(doc["action"], "stopRecord") == 0 ) {
        Serial.printf("StopRecord Action \n");
        attachServos();

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
  request->send(SPIFFS, "/index.html", "text/html");
}

// Callback: send style sheet
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

    delay(500);

    for(int i = 0; i < MAX_SERVOS; ++i) {

      calibrate[i].current = adc[i].readMiliVolts();
      calibrate[i].minPosition = min(calibrate[i].minPosition, calibrate[i].current);
      calibrate[i].maxPosition = max(calibrate[i].maxPosition, calibrate[i].current);
      calibrate[i].degree = map( calibrate[i].current, calibrate[i].minPosition, calibrate[i].maxPosition, 0, SERVO_DEGREE_MAX);
      
      Serial.printf("Servo: %d,\t W-degrees: %ld,\t delayed-analog: %u,\t delayed-degrees: %u, min: %u, max: %u \n", 
                    i, servoDegrees, calibrate[i].current, calibrate[i].degree, calibrate[i].minPosition, calibrate[i].maxPosition);
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

      if (servoRecordMode) {
        lastKnownDegrees = servoDegrees;
        pqi->current = servoDegrees;
        xQueueSend(qResponse, &pqi, portMAX_DELAY);  

      } else if(xSemaphoreTake(servoMutex, portMUX_NO_TIMEOUT) == pdTRUE) {
        servos[servo].write(servoDegrees);  
        xSemaphoreGive(servoMutex);
        
        vDelay = (int)(abs(servoDegrees - lastKnownDegrees)/60.0 * 100.0 + 50);
        delay( vDelay ); // 100ms/60degrees or 450ms for SERVO_DEGREE_MAX degrees

        calibrate[servo].current = adc[servo].readMiliVolts();;
        pqi->analog = calibrate[servo].current;
        pqi->current = map( calibrate[servo].current, calibrate[servo].minPosition, calibrate[servo].maxPosition, 0, SERVO_DEGREE_MAX);
        lastKnownDegrees = pqi->current;
        xQueueSend(qResponse, &pqi, portMAX_DELAY);        
        Serial.printf("Servo %d analog: %u, stackHighwater: %d, QueueDepth: %d\n", 
                       servo, pqi->analog, uxTaskGetStackHighWaterMark(NULL), uxQueueMessagesWaiting( requestQueue ));
      } 
    } else {
      vTaskDelay(1000/portTICK_PERIOD_MS);
    }
    pqi = NULL;
  }
  vTaskDelete( NULL );
}

void generateServoTasks() {
  static char const *pchTitle[4] = {"ServoTask-00", "ServoTask-01", "ServoTask-02", "ServoTask-03"};
    xTaskCreatePinnedToCore(
        servoActionResponse,  // Function that should be called
        "Queue Consumer",     // Name of the task (for debugging)
        2048,                 // Stack size (bytes)
        NULL,                 // Parameter to pass
        3,                    // Task priority
        NULL,                 // Task handle
        tskNO_AFFINITY        // Run on any available core
      );

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

void setup() {
    Serial.begin(115200);

    Serial.println("Skoona.net Multple Servos with Feedback.");

    servoMutex = xSemaphoreCreateMutex();

    qResponse = xQueueCreate( MAX_RESPONSE_QUEUE_SZ, sizeof( PQItem ) );
    if(qResponse == NULL) {
      Serial.printf("Error creating the response queue! \n");
    }

    for(int i = 0; i < MAX_SERVOS; ++i) {
      qRequest[i] = xQueueCreate( MAX_REQUEST_QUEUE_SZ, sizeof( PQItem ) );
      if(qRequest[i] == NULL) {
        Serial.printf("Error creating the request queue %d \n", i);
      }
        
      pinMode(servosPosition[i], INPUT);
      pinMode(servosPins[i], OUTPUT);

      adc[i].attach(servosPosition[i]);
      calibrate[i].minPosition = 155;
      calibrate[i].maxPosition = 3105;
    
      if(!servos[i].attach(servosPins[i],servoMtrChannel[i], 0, SERVO_DEGREE_MAX, SERVO_PULSE_MIN, SERVO_PULSE_MAX )) {
        Serial.printf("Servo %d attach error! \n", i);
      }
      servoRecordMode = false;
    }

    /*
     * Calibrate Servos
    */
    calibrateServos(135);
    calibrateServos(0);    
    calibrateServos(135);
    calibrateServos(SERVO_DEGREE_MAX);
    calibrateServos(135);

    /*
     * Create Servo Handler
     */
    generateServoTasks();

// Make sure we can read the file system
  if( !SPIFFS.begin()){
    Serial.println("Error mounting SPIFFS");
    while(1);
  }

 WiFi.begin(ssid, password);
  while ( WiFi.status() != WL_CONNECTED ) {
    delay(500);
    Serial.print(".");
  }
  online = true;
  // Print our IP address
  Serial.println("Connected!");
  Serial.print("My IP address: ");
  Serial.println(WiFi.localIP());

 /*
    // Start access point
    WiFi.softAP(ssid, password);
    
    // Print our IP address
    Serial.println();
    Serial.println("AP running");
    Serial.print("My IP address: ");
    Serial.println(WiFi.softAPIP());
  */

  // On HTTP request for root, provide index.html file
  server.on("/", HTTP_GET, onIndexRequest);

  // On HTTP Assets
  server.on("/jogDial.min.js", HTTP_GET, onJSRequest);
  server.on("/base_one_knob.png", HTTP_GET, onImgKnobRequest);
  server.on("/base_one_bg.png", HTTP_GET, onImgBgRequest);

  server.onNotFound(onPageNotFound);

  server.begin();

  // Start WebSocket server and assign callback
  webSocket.begin();
  webSocket.onEvent(onWebSocketEvent);
  

    Serial.printf("Setup Complete !\n");
}

void loop() {
  webSocket.loop();
}

/*
 Record Function:
 Start Timer for period x
  on period x
    - capture analog to degree
    - send degree to websocket for each servo
*/
