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
#define MAX_REQUEST_QUEUE_SZ 16        // Number of Request that can be queued

static const int    servosPwmPins[MAX_SERVOS] = {21, 19, 23, 18};
static const int    servosAdcPins[MAX_SERVOS] = {32, 33, 34, 35};
static const int  servoMtrChannel[MAX_SERVOS] = {0, 1, 2, 3};
static const int servoFdBKChannel[MAX_SERVOS] = {4, 5, 6, 7};

volatile unsigned long guiTimeBase = 0,        // default time base, target 0.5 seconds
    gulLastTimeBase = 0,                       // time base delta
    gulDisplayTimeBase = 0,                    // Display time base delta
    gulRecordInterval = 500,                   // Ticks interval 0.5 seconds
    gulDisplayInterval = 1250,                 // Ticks interval 1.25 seconds
    gulRecordCounter = 0;                      // Number of recorded records
volatile bool     gbRecordMode       = false,  // Recording
                  gvDuration         = false,  // 1/2 second marker
                  gbWSOnline         = false;  // WebSocket Client Connected
                  

bool gbCalibrationRequired = false;
int  giMaxRecordingCount   = 240;

volatile SemaphoreHandle_t servoMutex;

TaskHandle_t replayTaskHandle;

Servo servos[MAX_SERVOS];
ESP32AnalogRead adc[MAX_SERVOS];
QueueHandle_t qRequest[MAX_SERVOS];

ServoCalibration calibrate[MAX_SERVOS];

bool saveRecordedMovements(bool firstFlag, const uint8_t *payload);

bool loadServoStandards(void) {
  if ( !FileFS.exists(CALIBRATION_STANDARDS_FILE) ) {
    Serial.println("loadServoStandards(): Calibration file not found!");
    return false;
  }

  //file exists, reading and loading
  Serial.println("loadServoStandards(): Reading calibration standards");
  File calFile = FileFS.open(CALIBRATION_STANDARDS_FILE, FILE_READ);
  if (!calFile) {
    Serial.println("loadServoStandards(): Calibration  Failed to Open!");
    return false;
  }

  Serial.printf("loadServoStandards(): Opened calibration file, size = ");
  size_t calFileSize = calFile.size();
  Serial.println(calFileSize);

  Serial.printf("\nJSON parseObject() result : ");

  DynamicJsonDocument json(1024);
  DeserializationError err = deserializeJson(json, calFile);
  if ( err ) {
    Serial.println("Failed");
    calFile.close();
    return false;
  } 

  Serial.println("OK");

  if (json["Servos"]["calibrationRequired"]) {
      gbCalibrationRequired = json["Servos"]["calibrationRequired"];          
  }

  if (json["Servos"]["maxRecordingCount"]) {
      giMaxRecordingCount = json["Servos"]["maxRecordingCount"];          
  }
  
  char index[4][2] = {"0","1","2","3"};
  for (int idx = 0; idx < MAX_SERVOS; idx++) {
      strlcpy(calibrate[idx].name, json["Servos"][index[idx]]["name"] | "N/A", sizeof(calibrate[idx].name));
      calibrate[idx].maxDegree     = json["Servos"][index[idx]]["maxDegree"];
      calibrate[idx].minPosition   = json["Servos"][index[idx]]["minPosition"];
      calibrate[idx].maxPosition   = json["Servos"][index[idx]]["maxPosition"];
      calibrate[idx].minPulseWidth = json["Servos"][index[idx]]["minPulseWidth"];
      calibrate[idx].maxPulseWidth = json["Servos"][index[idx]]["maxPulseWidth"];
  }

  serializeJsonPretty(json, Serial);
  calFile.close();
    
  return true;
}

bool saveServoStandards(void)
{
  Serial.println("saveFileFSConfigFile(): Saving calibration file.");

  DynamicJsonDocument json(128);

  if (json["Servos"]["calibrationRequired"]) {
      json["Servos"]["calibrationRequired"] = gbCalibrationRequired;
  }

  if (json["Servos"]["maxRecordingCount"]) {
      json["Servos"]["maxRecordingCount"] = giMaxRecordingCount;
  }
  
  char index[4][2] = {"0","1","2","3"};
  for (int idx = 0; idx < MAX_SERVOS; idx++) {
    json["Servos"][index[idx]]["name"]          = calibrate[idx].name;
    json["Servos"][index[idx]]["maxDegree"]     = calibrate[idx].maxDegree;
    json["Servos"][index[idx]]["minPosition"]   = calibrate[idx].minPosition;
    json["Servos"][index[idx]]["maxPosition"]   = calibrate[idx].maxPosition ;
    json["Servos"][index[idx]]["minPulseWidth"] = calibrate[idx].minPulseWidth;
    json["Servos"][index[idx]]["maxPulseWidth"] = calibrate[idx].maxPulseWidth;
  }

  File calFile = FileFS.open(CALIBRATION_STANDARDS_FILE, FILE_WRITE);  // FILE_APPEND
  if (!calFile)
  {
    Serial.println("saveFileFSConfigFile(): Failed to open calibration file for writing.");
    return false;
  }

  serializeJsonPretty(json, Serial);
  serializeJson(json, calFile);

  calFile.close();
  //end save
  return true;
}

uint32_t readServoAnalog(int servo) {
  return adc[servo].readMiliVolts();
}
/*
 * Record generator
*/
void servoRecordRequest() {
    char recordBuffer[128];      // recording entries
    gulRecordCounter += 1;
    ServoCalibration recordings[MAX_SERVOS];

    memcpy(&recordings, &calibrate, sizeof(calibrate));

    // Serial.println("servoRecordRequest() Enter");

    if (gulRecordCounter > giMaxRecordingCount) {
      gbRecordMode = false;
      gulRecordCounter = 0;
      Serial.printf("Recording: [%lu] Limit Reached, Recording Stopped!\n", guiTimeBase);

      ws.printfAll("{\"action\":\"follow\"}" );
      return;
    }

    for(int servo = 0; servo < MAX_SERVOS; ++servo) {
      recordings[servo].analog = readServoAnalog( servo);
      recordings[servo].degree = map( recordings[servo].analog, recordings[servo].minPosition, recordings[servo].maxPosition, 0, recordings[servo].maxDegree);
      if (recordings[servo].analog >= recordings[servo].maxPosition) {
        recordings[servo].degree = recordings[servo].maxDegree;
      } if (recordings[servo].analog <= recordings[servo].minPosition) {
        recordings[servo].degree = 0;
      } else {
        recordings[servo].degree = recordings[servo].degree % recordings[servo].maxDegree;
      }
      snprintf(recordBuffer, sizeof(recordBuffer), "{\"action\":\"record\",\"servo\":%d,\"degree\":%u}", servo, recordings[servo].degree );
      Serial.printf("Recording: [%03lu] %s\n", gulRecordCounter, recordBuffer);
      ws.printfAll("%s", recordBuffer);
    }
    saveRecordedMovements(false, (const uint8_t *)&recordings );

  // Serial.println("servoRecordRequest() Exit");
}

bool servoRecordInterval(bool startStopFlag = false) {
  bool bUpdateDisplay =  false;

  guiTimeBase = millis();
  gvDuration = ((guiTimeBase - gulLastTimeBase) >= gulRecordInterval);
  bUpdateDisplay = ((guiTimeBase - gulDisplayTimeBase) >= gulDisplayInterval);

  // Serial.println("servoRecordInterval() Enter");
  if (gbWSOnline && gvDuration && gbRecordMode) {
    toggleLED();
    servoRecordRequest();
  }

  if (gvDuration) { // Every half second poll 
    gulLastTimeBase = guiTimeBase;        
  }

  if (bUpdateDisplay) {
    gulDisplayTimeBase = guiTimeBase;
    updateOLEDDisplay();
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
    if (!servos[i].attach(servosPwmPins[i], servoMtrChannel[i], 0, calibrate[i].maxDegree, calibrate[i].minPulseWidth, calibrate[i].maxPulseWidth))
    {
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
    Serial.println("calibrateServos() Enter");

    for(int i = 0; i < MAX_SERVOS; ++i) {
      servoDegrees = degrees % calibrate[i].maxDegree;
      if (servoDegrees == 0 && degrees == calibrate[i].maxDegree ) { 
        servoDegrees = calibrate[i].maxDegree;
      } else if (servoDegrees < 0) {
        servoDegrees = 0;
      }
      if (i == 0) {
        if (servoDegrees >= calibrate[i].maxDegree) {
          servos[i].write( servoDegrees / 3 );      
        } else if (servoDegrees == 0) {
          servos[i].write( servoDegrees );      
        } 
      } else {
        servos[i].write(servoDegrees);      
      }
    }

    vTaskDelay(pdMS_TO_TICKS(500));

    for(int i = 0; i < MAX_SERVOS; ++i) {
      calibrate[i].analog = readServoAnalog(i);
      calibrate[i].minPosition = min(calibrate[i].minPosition, calibrate[i].analog);
      calibrate[i].maxPosition = max(calibrate[i].maxPosition, calibrate[i].analog);
      calibrate[i].degree = map( calibrate[i].analog, calibrate[i].minPosition, calibrate[i].maxPosition, 0, calibrate[i].maxDegree);
      
      Serial.printf("calibrateServos(): %d,\t W-degrees: %ld,\t delayed-analog: %u,\t delayed-degrees: %u, min: %u, max: %u \n", 
                    i, servoDegrees, calibrate[i].analog, calibrate[i].degree, calibrate[i].minPosition, calibrate[i].maxPosition);
   } 

   Serial.println("calibrateServos() Exit");
}

void replayServoTask(void * _ptr) {
  uint32_t ulNotificationValue = 0;
  size_t size = 0, read = 0;
  int recordsToRead = 0;
  ServoCalibration recordings[MAX_SERVOS];

  memcpy(&recordings, &calibrate, sizeof(recordings));

  vTaskDelay(pdMS_TO_TICKS(1000));

  for(;;){ // infinite loop        

    ulNotificationValue = ulTaskNotifyTake( pdTRUE, portMAX_DELAY);
    if ( ulNotificationValue == 1 ) {
      Serial.printf("servoReplayTask() Active: %u\n", ulNotificationValue);

      vTaskDelay(pdMS_TO_TICKS(1000));

      if ( SPIFFS.exists(RECORDING_FILE) ) {
        //file exists, reading and loading
        Serial.printf("Reading file: %s\n", RECORDING_FILE);

        File file = SPIFFS.open(RECORDING_FILE, FILE_READ);
        if (file) {
          size = file.size();
          if (size < 1) {
            Serial.println("Recording File is EMPTY!");
            file.close();
            continue;
          } 

          recordsToRead = size / sizeof(recordings);
          Serial.printf("Recording File: size=%d, recordsToRead=%d\n", size, recordsToRead);
          while(recordsToRead != 0 && gbRecordMode) {
            toggleLED();
            read = file.readBytes((char *)&recordings, sizeof(recordings));
            recordsToRead -= 1;
            Serial.printf("Recording File: readBytes=%d\n", read);
            for(int i =0; i < MAX_SERVOS; i++) {
              servoActionRequest(i, recordings[i].degree);
            }
                        
            vTaskDelay(pdMS_TO_TICKS(500)); // replay in same period as recorded
          }

          file.close();      
        }
      }
    } else {
      Serial.printf("servoReplayTask() Timeout: %u\n", ulNotificationValue);
      vTaskDelay(pdMS_TO_TICKS(1000));
    }
    
    Serial.println("servoReplayTask() Waiting!");

  }
  vTaskDelete( NULL );
}

void servoTask(void * pServo) {
  PQItem pqi;
  int servo  = (int)(*(int *)pServo);
  int servoDegrees = 0,
      vDelay = 0,
      lastKnownDegrees[MAX_SERVOS] = {0,0,0,0};

  vTaskDelay(pdMS_TO_TICKS(1000));

  for(;;){ // infinite loop        
    if ( xQueueReceive(qRequest[servo], &pqi, portMAX_DELAY) == pdTRUE) {
      Serial.printf("servoTask() Enter: cfgDegreeMax=%u\n", calibrate[servo].maxDegree);
      if (NULL == pqi) {
        vTaskDelay(pdMS_TO_TICKS(1000));
        continue;
      }
      servoDegrees = pqi->degree % calibrate[servo].maxDegree;
      if (servoDegrees == 0 && pqi->degree == calibrate[servo].maxDegree ) { 
        servoDegrees = pqi->degree;
      } else if (servoDegrees < 1 ) {
        servoDegrees = 0;
      }

      if( !gbRecordMode && xSemaphoreTake(servoMutex, portMUX_NO_TIMEOUT) == pdTRUE) {
        servos[servo].write(servoDegrees);  
        xSemaphoreGive(servoMutex);
        
        vDelay = (int)(abs(servoDegrees - lastKnownDegrees[servo])/60.0 * 100.0 + 50);
        vTaskDelay( pdMS_TO_TICKS(vDelay) ); // 100ms/60degrees or 450ms for giDegreeMax degrees

        calibrate[servo].analog = readServoAnalog(servo);
        calibrate[servo].degree = map( calibrate[servo].analog, calibrate[servo].minPosition, calibrate[servo].maxPosition, 0, calibrate[servo].maxDegree);
        Serial.printf("Servo %d analog: %u, degree: %u, stackHighwater: %d, QueueDepth: %d\n", 
                       servo, calibrate[servo].analog, calibrate[servo].degree, uxTaskGetStackHighWaterMark(NULL), uxQueueMessagesWaiting( qRequest[servo] ));
      } 
      lastKnownDegrees[servo] = servoDegrees;
      heap_caps_free(pqi); // free allocation
    } else {
      vTaskDelay(pdMS_TO_TICKS(1000));
    }
    // Serial.println("servoTask() Exit");
    pqi = NULL;
  }
  vTaskDelete( NULL );
}

/*
 * Create Servo Handler
*/
void initServoTasks() {
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
      3072,              // Stack size (bytes)
      NULL,              // Parameter to pass
      2,                 // Task priority
      &replayTaskHandle, // Task handle
      tskNO_AFFINITY     // Run on any available core
  );

  // Serial.println("initServoTasks() Exited!");
}

void initServoControls() {
  size_t size = 0;
  bool servoCalibrate = true;

  servoMutex = xSemaphoreCreateMutex();
  
  File calibrateFile = FileFS.open(CALIBRATION_FILE, FILE_READ);  // FILE_APPEND
  if (calibrateFile) {
    size = calibrateFile.size();
    if (size > 64) {  // 64 is legacy size
      size = calibrateFile.readBytes((char *)&calibrate, sizeof(calibrate));
      servoCalibrate = false;   // exists  
    } else {
      Serial.printf("\ninitServoControls(a): Servos saved calibration read %d bytes.\n", size);
      loadServoStandards();    // loadStandards
      servoCalibrate = true;   // not saved, redo
    }
    calibrateFile.close();

    Serial.printf("\ninitServoControls(b): Servos saved calibration read %d bytes.\n", size);
  } else {
    Serial.println("\ninitServoControls(c): Failed to open saved calibrarion file for reading.");
    loadServoStandards();    // loadStandards
    servoCalibrate = true;   // not saved, redo
  }

  for(int i = 0; i < MAX_SERVOS; ++i) {
    qRequest[i] = xQueueCreate( MAX_REQUEST_QUEUE_SZ, sizeof( PQItem ) );
    if(qRequest[i] == NULL) {
      Serial.printf("Error creating the request queue %d \n", i);
    }
      
    pinMode(servosAdcPins[i], INPUT);
    pinMode(servosPwmPins[i], OUTPUT);

    adc[i].attach(servosAdcPins[i]);

    if (servoCalibrate && !gbCalibrationRequired) {    // skip if standards were read  
      calibrate[i].maxDegree = 90;
      calibrate[i].minPosition = 100;
      calibrate[i].maxPosition = 1010;
      calibrate[i].minPulseWidth = 500;
      calibrate[i].maxPulseWidth = 834;
    }

    if (!servos[i].attach(servosPwmPins[i], servoMtrChannel[i], 0, calibrate[i].maxDegree, calibrate[i].minPulseWidth, calibrate[i].maxPulseWidth))
    {
      Serial.printf("Servo %d attach error! \n", i);
    }
  }

  if (servoCalibrate && gbCalibrationRequired) {    
    for (int idx = 0; idx < MAX_SERVOS; idx++) {
        Serial.printf("Name: %s, %03u°, %04uµs, %04uµv, %04uµs, %04uµs\n",calibrate[idx].name,
        calibrate[idx].maxDegree,
        calibrate[idx].minPosition,
        calibrate[idx].maxPosition,
        calibrate[idx].minPulseWidth,
        calibrate[idx].maxPulseWidth);
    }

    /*
      * Calibrate Servos
    */
    calibrateServos(calibrate[1].maxDegree/2);
    calibrateServos(0);    
    calibrateServos(calibrate[1].maxDegree/2);
    calibrateServos(calibrate[1].maxDegree);
    calibrateServos(calibrate[1].maxDegree/2);

    Serial.println("Saving Servo Calibration!");
    calibrateFile = FileFS.open(CALIBRATION_FILE, FILE_WRITE);  // FILE_APPEND
    if (calibrateFile) {
      size = calibrateFile.write((const uint8_t*)&calibrate, sizeof(calibrate));
      calibrateFile.close();
      saveServoStandards();
      Serial.printf("\ninitServoControls(): Servos Saved Calibration wrote %d bytes.\n", size);
    } else {
      Serial.printf("\ninitServoControls(): Failed to open calibrarion file for writing\n");
    }
    Serial.println("\ninitServoControls(): Servo Calibration Completed!");
  }

  gbRecordMode = false;

  initServoTasks();

  Serial.println("initServoControls() Exited!");
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
              if (gbRecordMode) {
                attachServos();
              }
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

