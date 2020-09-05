/**
  * Servos.cpp
 */

#include "servos.h"
#include "main.h"


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
    if (gulRecordCounter > MAX_RECORD_COUNT) {
      gbRecordMode = false;
      gulRecordCounter = 0;
      Serial.printf("Recording: [%lu] Limit Reached, Recording Stopped!\n", guiTimeBase);

      snprintf(recordBuffer, sizeof(recordBuffer), "{\"action\":\"follow\"}" );
      webSocket.broadcastTXT( recordBuffer, strlen(recordBuffer), false );
      return;
    }

    for(int servo = 0; servo < MAX_SERVOS; ++servo) {
      calibrate[servo].current = readServoAnalog( servo);
      calibrate[servo].degree = map( calibrate[servo].current, calibrate[servo].minPosition, calibrate[servo].maxPosition, 0, SERVO_DEGREE_MAX);
      if (calibrate[servo].current >= calibrate[servo].maxPosition) {
        calibrate[servo].degree = SERVO_DEGREE_MAX;
      } if (calibrate[servo].current <= calibrate[servo].minPosition) {
        calibrate[servo].degree = 0;
      } else {
        calibrate[servo].degree = calibrate[servo].degree % (SERVO_DEGREE_MAX + 1);
      }
      snprintf(recordBuffer, sizeof(recordBuffer), "{\"action\":\"record\",\"servo\":%d,\"degree\":%u}", servo, calibrate[servo].degree );
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
      calibrate[i].current = readServoAnalog(i);
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

  vTaskDelay(1000/portTICK_PERIOD_MS);

  for(;;){ // infinite loop        
    if ( xQueueReceive(qRequest[servo], &pqi, portMAX_DELAY) == pdTRUE) {
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

        calibrate[servo].current = readServoAnalog(servo);
        calibrate[servo].degree = map( calibrate[servo].current, calibrate[servo].minPosition, calibrate[servo].maxPosition, 0, SERVO_DEGREE_MAX);
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
    
      if(!servos[i].attach(servosPins[i],servoMtrChannel[i], 0, SERVO_DEGREE_MAX, SERVO_PULSE_MIN, SERVO_PULSE_MAX )) {
        Serial.printf("Servo %d attach error! \n", i);
      }
    }
    /*
      * Calibrate Servos
    */
    calibrateServos(135);
    calibrateServos(0);    
    calibrateServos(135);
    calibrateServos(SERVO_DEGREE_MAX);
    calibrateServos(135);

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
