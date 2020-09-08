/**
  * Servos.cpp
 */

#include <Arduino.h>
#include <FS.h>
#include <SPIFFS.h>
#include <Servo.h>
#include <ESP32AnalogRead.h>

#include "servos.h"
extern FS* filesystem;
#define FileFS  SPIFFS
#define RECORDING_FILE "/records.bin"
#define CALIBRATION_FILE "/servos.bin"
#define MAX_REQUEST_QUEUE_SZ 24        // Number of Request that can be queued
#define LED_BUILTIN       22

static const int    servosPwmPins[MAX_SERVOS] = {21, 19, 23, 18};
static const int    servosAdcPins[MAX_SERVOS] = {32, 33, 34, 35};
static const int  servoMtrChannel[MAX_SERVOS] = {0, 1, 2, 3};
static const int servoFdBKChannel[MAX_SERVOS] = {4, 5, 6, 7};

volatile unsigned long guiTimeBase   = 0,      // default time base, target 0.5 seconds
                  gulLastTimeBase    = 0,      // time base delta
                  gulRecordInterval  = 500,    // Ticks interval 0.5 seconds
                  gulRecordCounter   = 0;      // Number of recorded records
volatile bool     gbRecordMode       = false,  // Recording
                  gvDuration         = false,  // 1/2 second marker
                  gbWSOnline         = false;  // WebSocket Client Connected


volatile SemaphoreHandle_t servoMutex;

TaskHandle_t replayTaskHandle;

Servo servos[MAX_SERVOS];
ESP32AnalogRead adc[MAX_SERVOS];
QueueHandle_t qRequest[MAX_SERVOS];

ServoCalibration calibrate[MAX_SERVOS];

bool saveRecordedMovements(bool firstFlag, const uint8_t *payload);
bool replayRecordedMovements();


uint32_t readServoAnalog(int servo) {
  // Serial.println("readServoAnalog() Enter");
  uint32_t res = adc[servo].readMiliVolts();
  // Serial.println("readServoAnalog() Exit");
    return res;
}
/*
 * Record generator
*/
void servoRecordRequest() {
    char recordBuffer[128];      // recording entries
    gulRecordCounter += 1;

    // Serial.println("servoRecordRequest() Enter");

    if (gulRecordCounter > giMaxRecCnt) {
      gbRecordMode = false;
      gulRecordCounter = 0;
      Serial.printf("Recording: [%lu] Limit Reached, Recording Stopped!\n", guiTimeBase);

      ws.printfAll("{\"action\":\"follow\"}" );
      return;
    }

    for(int servo = 0; servo < MAX_SERVOS; ++servo) {
      calibrate[servo].current = readServoAnalog( servo);
      calibrate[servo].degree = map( calibrate[servo].current, calibrate[servo].minPosition, calibrate[servo].maxPosition, 0, giDegreeMax);
      if (calibrate[servo].current >= calibrate[servo].maxPosition) {
        calibrate[servo].degree = giDegreeMax;
      } if (calibrate[servo].current <= calibrate[servo].minPosition) {
        calibrate[servo].degree = 0;
      } else {
        calibrate[servo].degree = calibrate[servo].degree % (giDegreeMax + 1);
      }
      snprintf(recordBuffer, sizeof(recordBuffer), "{\"action\":\"record\",\"servo\":%d,\"degree\":%u}", servo, calibrate[servo].degree );
      Serial.printf("Recording: [%lu] %s\n", guiTimeBase, recordBuffer);
      ws.printfAll("%s", recordBuffer);
    }
    saveRecordedMovements(false, (const uint8_t *)&calibrate );

  // Serial.println("servoRecordRequest() Exit");
}

bool servoRecordInterval(bool startStopFlag = false) {
  guiTimeBase = millis();
  gvDuration = ((guiTimeBase - gulLastTimeBase) >= gulRecordInterval);
  // Serial.println("servoRecordInterval() Enter");
  if (gbWSOnline && gvDuration && gbRecordMode) {
    digitalWrite(LED_BUILTIN, HIGH);
    servoRecordRequest();
    digitalWrite(LED_BUILTIN, LOW);
  }

  if (gvDuration) { // Every half second poll 
    gulLastTimeBase = guiTimeBase;        
  }

  // Serial.println("servoRecordInterval() Exit");
  return gvDuration;
}

/*
 * Function to send new request
*/
void servoActionRequest(int servo, int degrees) {    
    PQItem pqi = NULL;
    // Serial.println("servoActionRequest() Enter");

    pqi = (PQItem) heap_caps_malloc(sizeof( QItem ), MALLOC_CAP_8BIT);  // heap_caps_free() or free() later
    if (pqi != NULL) {
      pqi->servo  = servo;
      pqi->degree = degrees;
      xQueueSend(qRequest[servo], &pqi, portMAX_DELAY);
    } else {
      Serial.printf("Servo %d Memory Allocation Failed! pqi=%p .....................\n", servo, pqi);
    }
    // Serial.println("servoActionRequest() Exit");
}

/*
 * function to re-attach servos
*/
bool attachServos() {
  bool rc = true;
  gbRecordMode = false;
  for(int i = 0; i < MAX_SERVOS; ++i) {
    if(!servos[i].attach(servosPwmPins[i],servoMtrChannel[i], 0, giDegreeMax, giMinPulseWidth, giMaxPulseWidth )) {
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
    // Serial.println("calibrateServos() Enter");

    for(int i = 0; i < MAX_SERVOS; ++i) {
      servoDegrees = degrees % giDegreeMax;
      if (servoDegrees == 0 && degrees == giDegreeMax ) { 
        servoDegrees = degrees;
      }
      servos[i].write(servoDegrees);      
    }

    vTaskDelay(500/portTICK_PERIOD_MS);

    for(int i = 0; i < MAX_SERVOS; ++i) {
      calibrate[i].current = readServoAnalog(i);
      calibrate[i].minPosition = min(calibrate[i].minPosition, calibrate[i].current);
      calibrate[i].maxPosition = max(calibrate[i].maxPosition, calibrate[i].current);
      calibrate[i].degree = map( calibrate[i].current, calibrate[i].minPosition, calibrate[i].maxPosition, 0, giDegreeMax);
      
      Serial.printf("Servo: %d,\t W-degrees: %ld,\t delayed-analog: %u,\t delayed-degrees: %u, min: %u, max: %u \n", 
                    i, servoDegrees, calibrate[i].current, calibrate[i].degree, calibrate[i].minPosition, calibrate[i].maxPosition);
   } 

  //  Serial.println("calibrateServos() Exit");
}

void replayServoTask(void * _ptr) {

  uint32_t ulNotificationValue = 0;
  vTaskDelay(1000/portTICK_PERIOD_MS);

  for(;;){ // infinite loop        

    ulNotificationValue = ulTaskNotifyTake( pdTRUE, portMAX_DELAY);
    if ( ulNotificationValue == 1 ) {
      Serial.printf("servoReplayTask() Active: %u", ulNotificationValue);

      vTaskDelay(1000/portTICK_PERIOD_MS);

      replayRecordedMovements();
    } else {
      Serial.printf("servoReplayTask() Timeout: %u", ulNotificationValue);
      vTaskDelay(1000/portTICK_PERIOD_MS);
    }
    
    Serial.println("servoReplayTask() Exit");

  }
  vTaskDelete( NULL );
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
      // Serial.println("servoTask() Enter");
      if (NULL == pqi) {
        vTaskDelay(1000/portTICK_PERIOD_MS);
        continue;
      }
      servoDegrees = pqi->degree % giDegreeMax;
      if (servoDegrees == 0 && pqi->degree == giDegreeMax ) { 
        servoDegrees = pqi->degree;
      } 

      if( !gbRecordMode && xSemaphoreTake(servoMutex, portMUX_NO_TIMEOUT) == pdTRUE) {
        servos[servo].write(servoDegrees);  
        xSemaphoreGive(servoMutex);
        
        vDelay = (int)(abs(servoDegrees - lastKnownDegrees)/60.0 * 100.0 + 50);
        vTaskDelay(vDelay/portTICK_PERIOD_MS); // 100ms/60degrees or 450ms for giDegreeMax degrees

        calibrate[servo].current = readServoAnalog(servo);
        calibrate[servo].degree = map( calibrate[servo].current, calibrate[servo].minPosition, calibrate[servo].maxPosition, 0, giDegreeMax);
        Serial.printf("Servo %d analog: %u, degree: %u, stackHighwater: %d, QueueDepth: %d\n", 
                       servo, calibrate[servo].current, calibrate[servo].degree, uxTaskGetStackHighWaterMark(NULL), uxQueueMessagesWaiting( qRequest[servo] ));
      } 
      lastKnownDegrees = servoDegrees;
      heap_caps_free(pqi); // free allocation
    } else {
      vTaskDelay(1000/portTICK_PERIOD_MS);
    }
    // Serial.println("servoTask() Exit");
    pqi = NULL;
  }
  vTaskDelete( NULL );
}

/*
 * Create Servo Handler
*/
void initializeServoTasks() {
  static const char *pchTitle[5] = {"ServoTask-00", "ServoTask-01", "ServoTask-02", "ServoTask-03", "ReplayServoTask"};
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

  xTaskCreatePinnedToCore(
      replayServoTask,   // Function that should be called
      pchTitle[4],       // Name of the task (for debugging)
      2048,              // Stack size (bytes)
      NULL,              // Parameter to pass
      2,                 // Task priority
      &replayTaskHandle, // Task handle
      tskNO_AFFINITY     // Run on any available core
  );

  // Serial.println("initializeServoTasks() Exited!");
}

void initializeServoControls() {
  size_t size = 0;
  bool gbServoCalibrate = true;

  servoMutex = xSemaphoreCreateMutex();
  
  File calibrateFile = FileFS.open(CALIBRATION_FILE, FILE_READ);  // FILE_APPEND
  if (calibrateFile) {
    size = calibrateFile.size();
    if (size > 0) {
      size = calibrateFile.readBytes((char *)&calibrate, sizeof(calibrate));
      gbServoCalibrate = false;   // exists  
    }
    calibrateFile.close();

    Serial.printf("Servos Saved Calibration Read %d bytes.\n", size);
  } else {
    Serial.println("Failed to open calibrarion file for reading");
    gbServoCalibrate = true;   // not saved, redo
  }

  for(int i = 0; i < MAX_SERVOS; ++i) {
    qRequest[i] = xQueueCreate( MAX_REQUEST_QUEUE_SZ, sizeof( PQItem ) );
    if(qRequest[i] == NULL) {
      Serial.printf("Error creating the request queue %d \n", i);
    }
      
    pinMode(servosAdcPins[i], INPUT);
    pinMode(servosPwmPins[i], OUTPUT);

    adc[i].attach(servosAdcPins[i]);

    if (!gbServoCalibrate) {      
      calibrate[i].minPosition = 155;
      calibrate[i].maxPosition = 3105;
    }
  
    if(!servos[i].attach(servosPwmPins[i],servoMtrChannel[i], 0, giDegreeMax, giMinPulseWidth, giMaxPulseWidth )) {
      Serial.printf("Servo %d attach error! \n", i);
    }
  }

  if (gbServoCalibrate) {    
    /*
      * Calibrate Servos
    */
    calibrateServos(giDegreeMax/2);
    calibrateServos(0);    
    calibrateServos(giDegreeMax/2);
    calibrateServos(giDegreeMax);
    calibrateServos(giDegreeMax/2);

    Serial.println("Saving Servo Calibration!");
    calibrateFile = FileFS.open(CALIBRATION_FILE, FILE_WRITE);  // FILE_APPEND
    if (calibrateFile) {
      size = calibrateFile.write((const uint8_t*)&calibrate, sizeof(calibrate));
      calibrateFile.close();
      Serial.printf("Servos Saved Calibration wrote %d bytes.\n", size);
    } else {
      Serial.printf("Failed to open calibrarion file for writing\n");
    }
    Serial.println("Servo Calibration Completed!");
  }

  gbRecordMode = false;

  initializeServoTasks();

  // Serial.println("initializeServoControls() Exited!");
}

bool replayRecordedMovements() {
  size_t size = 0, read = 0;
  int recordsToRead = 0;
  ServoCalibration recordings[MAX_SERVOS];

  if ( SPIFFS.exists(RECORDING_FILE) ) {
    //file exists, reading and loading
    Serial.printf("Reading file: %s\n", RECORDING_FILE);

    File file = SPIFFS.open(RECORDING_FILE, FILE_READ);
    if (file) {
      size = file.size();
      if (size < 1) {
        Serial.println("Recording File is EMPTY!");
        file.close();
        return false;
      } 

      recordsToRead = size / sizeof(recordings);
      Serial.printf("Recording File: size=%d, recordsToRead=%d\n", size, recordsToRead);
      while(recordsToRead != 0) {
        read = file.readBytes((char *)&recordings, sizeof(recordings));
        recordsToRead -= 1;
        Serial.printf("Recording File: readBytes=%d\n", read);
        for(int i =0; i < MAX_SERVOS; i++) {
          servoActionRequest(i, recordings[i].degree);
        }
        vTaskDelay(500/portTICK_PERIOD_MS); // replay in same period as recorded
      }

      file.close();      
    }
  } else {
    return false;
  }
  return true;
}

bool saveRecordedMovements(bool firstFlag, const uint8_t *payload) {
  File file;
  if (firstFlag) {
    file = SPIFFS.open(RECORDING_FILE, FILE_WRITE);
  } else {
    file = SPIFFS.open(RECORDING_FILE, FILE_APPEND);
  }
  if (!file) {
    Serial.printf("Failed to open file: %s, for writing. \n", RECORDING_FILE);
    return false;
  }

  file.write(payload, sizeof(calibrate));
  file.flush();
  file.close();
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
            server->printfAll("{\"action\":\"follow\"}");
            xTaskNotifyGive(replayTaskHandle); // release the BG Task

          } else if ( strcmp(doc["action"], "startRecord") == 0 ) {
            Serial.printf("StartRecord Action \n");
            if (!gbRecordMode) {
              detachServos();
            }
            saveRecordedMovements(true, (const uint8_t *)&calibrate );
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

