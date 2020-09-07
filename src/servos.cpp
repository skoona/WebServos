/**
  * Servos.cpp
 */

#include <Arduino.h>
#include <FS.h>
#include <SPIFFS.h>
#include <Servo.h>
#include <ESP32AnalogRead.h>
#include <Preferences.h>

#include "servos.h"

static const int       servosPins[MAX_SERVOS] = {21, 19, 23, 18};
static const int   servosPosition[MAX_SERVOS] = {32, 33, 34, 35};
static const int  servoMtrChannel[MAX_SERVOS] = {0, 1, 2, 3};
static const int servoFdBKChannel[MAX_SERVOS] = {4, 5, 6, 7};

volatile unsigned long guiTimeBase   = 0,      // default time base, target 0.5 seconds
                  gulLastTimeBase    = 0,      // time base delta
                  gulRecordInterval  = 500,    // Ticks interval 0.5 seconds
                  gulRecordCounter   = 0;      // Number of recorded records
volatile bool     gbRecordMode       = false,  // Recording
                  gvDuration         = false,  // 1/2 second marker
                  gbWSOnline         = false,  // WebSocket Client Connected
                  gbCalibrate        = true,   // Servo Calibration Required
                  gbServoCalibrate   = true;   // isCalibration Required
int servoDegreeMax = 270;
int servoRecordMax = 240;


volatile SemaphoreHandle_t servoMutex;
Servo servos[MAX_SERVOS];
ESP32AnalogRead adc[MAX_SERVOS];
QueueHandle_t qRequest[MAX_SERVOS];
Preferences preferences;
ServoCalibration calibrate[MAX_SERVOS];

uint32_t readServoAnalog(int servo) {
    return adc[servo].readMiliVolts();
}
/*
 * Record generator
*/
void servoRecordRequest() {
    char recordBuffer[128];      // recording entries
    gulRecordCounter += 1;
    if (gulRecordCounter > servoRecordMax) {
      gbRecordMode = false;
      gulRecordCounter = 0;
      Serial.printf("Recording: [%lu] Limit Reached, Recording Stopped!\n", guiTimeBase);

      ws.printfAll("{\"action\":\"follow\"}" );
      return;
    }

    for(int servo = 0; servo < MAX_SERVOS; ++servo) {
      calibrate[servo].current = readServoAnalog( servo);
      calibrate[servo].degree = map( calibrate[servo].current, calibrate[servo].minPosition, calibrate[servo].maxPosition, 0, servoDegreeMax);
      if (calibrate[servo].current >= calibrate[servo].maxPosition) {
        calibrate[servo].degree = servoDegreeMax;
      } if (calibrate[servo].current <= calibrate[servo].minPosition) {
        calibrate[servo].degree = 0;
      } else {
        calibrate[servo].degree = calibrate[servo].degree % (servoDegreeMax + 1);
      }
      snprintf(recordBuffer, sizeof(recordBuffer), "{\"action\":\"record\",\"servo\":%d,\"degree\":%u}", servo, calibrate[servo].degree );
      Serial.printf("Recording: [%lu] %s\n", guiTimeBase, recordBuffer);
      ws.printfAll("%s", recordBuffer);
    }
}

bool servoRecordInterval(bool startStopFlag = false) {
  guiTimeBase = millis();
  gvDuration = ((guiTimeBase - gulLastTimeBase) >= gulRecordInterval);

  if (gbWSOnline && gvDuration && gbRecordMode) {
    servoRecordRequest();
  }

  if (gvDuration) { // Every half second poll 
    gulLastTimeBase = guiTimeBase;        
  }

  return gvDuration;
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
    if(!servos[i].attach(servosPins[i],servoMtrChannel[i], 0, servoDegreeMax, SERVO_PULSE_MIN, SERVO_PULSE_MAX )) {
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

void calibrateServos(long degrees) {
    long servoDegrees = 0;
    
    for(int i = 0; i < MAX_SERVOS; ++i) {
      servoDegrees = degrees % servoDegreeMax;
      if (servoDegrees == 0 && degrees == servoDegreeMax ) { 
        servoDegrees = degrees;
      }
      servos[i].write(servoDegrees);      
    }

    vTaskDelay(500/portTICK_PERIOD_MS);

    for(int i = 0; i < MAX_SERVOS; ++i) {
      calibrate[i].current = readServoAnalog(i);
      calibrate[i].minPosition = min(calibrate[i].minPosition, calibrate[i].current);
      calibrate[i].maxPosition = max(calibrate[i].maxPosition, calibrate[i].current);
      calibrate[i].degree = map( calibrate[i].current, calibrate[i].minPosition, calibrate[i].maxPosition, 0, servoDegreeMax);
      
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

  vTaskDelay(1000/portTICK_PERIOD_MS);

  for(;;){ // infinite loop        
    if ( xQueueReceive(qRequest[servo], &pqi, portMAX_DELAY) == pdTRUE) {
      if (NULL == pqi) {
        vTaskDelay(1000/portTICK_PERIOD_MS);
        continue;
      }
      servoDegrees = pqi->degree % servoDegreeMax;
      if (servoDegrees == 0 && pqi->degree == servoDegreeMax ) { 
        servoDegrees = pqi->degree;
      } 

      if( !gbRecordMode && xSemaphoreTake(servoMutex, portMUX_NO_TIMEOUT) == pdTRUE) {
        servos[servo].write(servoDegrees);  
        xSemaphoreGive(servoMutex);
        
        vDelay = (int)(abs(servoDegrees - lastKnownDegrees)/60.0 * 100.0 + 50);
        vTaskDelay(vDelay/portTICK_PERIOD_MS); // 100ms/60degrees or 450ms for servoDegreeMax degrees

        calibrate[servo].current = readServoAnalog(servo);
        calibrate[servo].degree = map( calibrate[servo].current, calibrate[servo].minPosition, calibrate[servo].maxPosition, 0, servoDegreeMax);
        Serial.printf("Servo %d analog: %u, degree: %u, stackHighwater: %d, QueueDepth: %d\n", 
                       servo, calibrate[servo].current, calibrate[servo].degree, uxTaskGetStackHighWaterMark(NULL), uxQueueMessagesWaiting( qRequest[servo] ));
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

/*
 * Create Servo Handler
*/
void initializeServoTasks() {
  static const char *pchTitle[4] = {"ServoTask-00", "ServoTask-01", "ServoTask-02", "ServoTask-03"};
  static int  servoIds[MAX_SERVOS] = {0,1,2,3};

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
  Serial.println("initializeServoTasks() Exited!");
}

void initializeServoControls() {
  size_t size = 0;

  servoMutex = xSemaphoreCreateMutex();

  gbCalibrate = preferences.begin("Calibration", false);
  if ( gbCalibrate ) {
    Serial.println("Servo Calibration Preferences Opened!");
    size = preferences.getBytes("Servos", &calibrate, sizeof(calibrate));
    if (size > 0 ) {
      servoDegreeMax = preferences.getInt("DegreeMax", servoDegreeMax);
      servoRecordMax = preferences.getInt("RecordMax", servoRecordMax);
      gbServoCalibrate = false;   // exists  
      Serial.printf("Servos Preferences Read %d bytes.\n", size);
    } else {
      gbServoCalibrate = true;   // not saved, redo
      Serial.println("Servo Preferences Not Found");
    }
  }

  if (gbServoCalibrate) {
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
    
      if(!servos[i].attach(servosPins[i],servoMtrChannel[i], 0, servoDegreeMax, SERVO_PULSE_MIN, SERVO_PULSE_MAX )) {
        Serial.printf("Servo %d attach error! \n", i);
      }
    }
    /*
      * Calibrate Servos
    */
    calibrateServos(servoDegreeMax/2);
    calibrateServos(0);    
    calibrateServos(servoDegreeMax/2);
    calibrateServos(servoDegreeMax);
    calibrateServos(servoDegreeMax/2);

    if (gbCalibrate) {
      Serial.println("Saving Servo Calibration!");
      size = preferences.putBytes("Servos", &calibrate, sizeof(calibrate));
        preferences.putInt("DegreeMax", servoDegreeMax);
        preferences.putInt("RecordMax", servoRecordMax);
      Serial.printf("Saved %d bytes! \n", size);  
    }
    Serial.println("Servo Calibration Completed!");
  }

  if (gbCalibrate) {
    preferences.end();
    Serial.println("Servo Calibration Preferences Closed!");
  }
  gbRecordMode = false;

  initializeServoTasks();

  Serial.println("initializeServoControls() Exited!");
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


void onEvent( AsyncWebSocket * server, 
              AsyncWebSocketClient * client, 
              AwsEventType type, 
              void * arg, 
              uint8_t *data, 
              size_t len) {

  static char lastBuffer[128];   // filter duplicate messages
  static String lastStateBuffer;   // Command State
  DynamicJsonDocument doc(200);
  DeserializationError errorJson;
  AwsFrameInfo *info = (AwsFrameInfo *)arg;

  switch (type) {
    case WS_EVT_CONNECT:
      //client connected
      Serial.printf("ws[%s][%u] Connect from %s\n", server->url(), client->id(), client->remoteIP().toString().c_str());
      gbWSOnline = true;
      client->ping();
      break;
    case WS_EVT_DISCONNECT:
      //client disconnected
      gbWSOnline = false;
      Serial.printf("ws[%s][%u] Disconnect! \n", server->url(), client->id());
      break;
    case WS_EVT_PONG:
      //pong message was received (in response to a ping request maybe)
      Serial.printf("ws[%s][%u] pong[%u]: %s\n", server->url(), client->id(), len, (len)?(char*)data:"");
      break;
    case WS_EVT_ERROR:
      //error was received from the other end
      Serial.printf("ws[%s][%u] error(%u): %s\n", server->url(), client->id(), *((uint16_t*)arg), (char*)data);
      break;
    case WS_EVT_DATA:
      //data packet
      if(info->final && info->index == 0 && info->len == len){
        //the whole message is in a single frame and we got all of it's data
        Serial.printf("ws[%s][%u] %s-message[%llu]: ", server->url(), client->id(), (info->opcode == WS_TEXT)?"text":"binary", info->len);
        if(info->opcode == WS_TEXT){
          data[len] = 0;
          Serial.printf("%s\n", (char*)data);
          if ( strcmp((char *)data, lastBuffer) == 0 && strcmp((char *)data, "{\"action\":\"currentState\"}") != 0) { // duplicate message
            Serial.printf("ws[%s][%u] Duplicate Message Received text: %s\n", server->url(), client->id(), data);
            break;
          } else {
            strncpy(lastBuffer, (char *)data, sizeof(lastBuffer));
          }
          if (lastStateBuffer.isEmpty()) {
            lastStateBuffer = "{\"action\":\"state\",\"button\":\"follow\"}";
          }
          // Print out raw message
          Serial.printf("[%u] Received text: %s\n", client->id(), data);
    
          errorJson = deserializeJson(doc, data);
          if(errorJson) {
            Serial.printf("deserializeJson() failed! cause=%s\n", errorJson.c_str());
            break;
          }

          // determine action
          if ( strcmp(doc["action"], "currentState") == 0 ) { // move to neutral positon -- currentState
            Serial.printf("Current State Action \n");
            for(int i = 0; i < MAX_SERVOS; ++i) {
              server->printfAll("{\"action\":\"state\",\"servo\":%d,\"degree\":%d}", i, calibrate[i].degree);
              Serial.printf("Sending: {\"action\":\"state\",\"servo\":%d,\"degree\":%d}\n", i, calibrate[i].degree);
            }
            server->printfAll("%s", lastStateBuffer.c_str());

          } else if ( strcmp(doc["action"], "follow") == 0 ) {
            Serial.printf("Follow Action \n");
            if ( !gbRecordMode && doc.containsKey("degree") ) {
              servoActionRequest(doc["servo"], doc["degree"]);        
            } else if (!doc.containsKey("degree")) {
              attachServos();
              server->printfAll("{\"action\":\"follow\"}");
              lastStateBuffer = "{\"action\":\"state\",\"button\":\"follow\"}";
            }

          } else if ( strcmp(doc["action"], "play") == 0 ) {
            Serial.printf("Play Action \n");
            if(gbRecordMode) {
              attachServos();
            }
            server->printfAll("{\"action\":\"play\"}");
            lastStateBuffer = "{\"action\":\"state\",\"button\":\"play\"}";

          } else if ( strcmp(doc["action"], "startRecord") == 0 ) {
            Serial.printf("StartRecord Action \n");
            if (!gbRecordMode) {
              detachServos();
            }
            server->printfAll("{\"action\":\"startRecord\"}");
            lastStateBuffer = "{\"action\":\"state\",\"button\":\"startRecord\"}";

          } else if ( strcmp(doc["action"], "stopRecord") == 0 ) {
            Serial.printf("StopRecord Action \n");
            if(gbRecordMode) {
              attachServos();
            }
            server->printfAll("{\"action\":\"stopRecord\"}");
            lastStateBuffer = "{\"action\":\"state\",\"button\":\"stopRecord\"}";

          } else {
            Serial.println("Message not recognized!");
          }

        } else {
          Serial.printf("Unsupported Binary Message; containing: ");
          for(size_t i=0; i < info->len; i++){
            Serial.printf("%02x ", data[i]);
          }
          Serial.printf("\n");
          break;
        }
      } else {
        //message is comprised of multiple frames or the frame is split into multiple packets
        if(info->index == 0){
          if(info->num == 0)
            Serial.printf("ws[%s][%u] %s-message start\n", server->url(), client->id(), (info->message_opcode == WS_TEXT)?"text":"binary");

          Serial.printf("ws[%s][%u] frame[%u] start[%llu]\n", server->url(), client->id(), info->num, info->len);
        }

        Serial.printf("ws[%s][%u] frame[%u] %s[%llu - %llu]: ", server->url(), client->id(), info->num, (info->message_opcode == WS_TEXT)?"text":"binary", info->index, info->index + len);
        if(info->message_opcode == WS_TEXT){
          data[len] = 0;
          Serial.printf("%s\n", (char*)data);
        } else {
          for(size_t i=0; i < len; i++){
            Serial.printf("%02x ", data[i]);
          }
          Serial.printf("\n");
        }

        if((info->index + len) == info->len){
          Serial.printf("ws[%s][%u] frame[%u] end[%llu]\n", server->url(), client->id(), info->num, info->len);
          if(info->final)
            Serial.printf("ws[%s][%u] %s-message end\n", server->url(), client->id(), (info->message_opcode == WS_TEXT)?"text":"binary");

        }
      }        
      break;

    default:
      break;
  }
}

