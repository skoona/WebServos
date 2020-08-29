#include "Arduino.h"
#include <Servo.h>
#include <ESP32AnalogRead.h>

#define MAX_SERVOS 4
#define MAX_QUEUE_SZ 64
#define SERVO_PULSE_MIN 500
#define SERVO_PULSE_MAX 2500
#define SERVO_DEGREE_MAX 270

static int servoIds[MAX_SERVOS] = {0,1,2,3};
static const int servosPins[MAX_SERVOS] = {21, 19, 23, 18};
static const int servosPosition[MAX_SERVOS] = {32, 33, 34, 35};

static const int servoMtrChannel[MAX_SERVOS] = {0, 1, 2, 3};
static const int servoFdBKChannel[MAX_SERVOS] = {4, 5, 6, 7};

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
  uint32_t minPosition;
  uint32_t maxPosition;
} ServoPositions, *PServoPostions;


Servo servos[MAX_SERVOS];
ESP32AnalogRead adc[MAX_SERVOS];
ServoPositions calibrate[MAX_SERVOS];

uint32_t readAnalog(int servo, bool calibration = false) {
  uint32_t res = adc[servo].readMiliVolts();
  calibrate[servo].current = res;
  if (calibration) {
    calibrate[servo].minPosition = min(calibrate[servo].minPosition, res);
    calibrate[servo].maxPosition = max(calibrate[servo].maxPosition, res);
  }
  return res;
}

void calibrateServos(long degrees) {
    long servoPositionPost = 0;
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

      readAnalog(i, true);
      servoPositionPost = map( calibrate[i].current, calibrate[i].minPosition, calibrate[i].maxPosition, 0, SERVO_DEGREE_MAX);

      Serial.printf("Servo: %d,\t W-degrees: %ld,\t delayed-analog: %u,\t delayed-degrees: %ld, min: %u, max: %u \n", 
                    i, servoDegrees, calibrate[i].current, servoPositionPost, calibrate[i].minPosition, calibrate[i].maxPosition);
   } 
}

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

void servoActionResponse(void * _unused) {
  PQItem pqi = NULL;
  while ( true) {
    if( xQueueReceive(qResponse, &pqi, portMAX_DELAY) == pdTRUE ) {
      /* Do something useful with pqi */
      heap_caps_free(pqi);
    }
  }
  vTaskDelete( NULL );
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
      if(xSemaphoreTake(servoMutex, portMUX_NO_TIMEOUT) == pdTRUE) {
        servos[servo].write(servoDegrees);  
        xSemaphoreGive(servoMutex);
        
        vDelay = (int)(abs(servoDegrees - lastKnownDegrees)/60.0 * 100.0 + 50);
        delay( vDelay ); // 100ms/60degrees or 450ms for SERVO_DEGREE_MAX degrees

        pqi->analog = readAnalog(servo, false);
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
        1024,                 // Stack size (bytes)
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

    qResponse = xQueueCreate( MAX_QUEUE_SZ, sizeof( PQItem ) );
    if(qResponse == NULL) {
      Serial.printf("Error creating the response queue! \n");
    }

    for(int i = 0; i < MAX_SERVOS; ++i) {
      qRequest[i] = xQueueCreate( MAX_QUEUE_SZ, sizeof( PQItem ) );
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

    /*
     * Create Servo Handler
     */
    generateServoTasks();

    Serial.printf("Setup Complete !\n");
}

void loop() {
    for(long posDegrees = 0; posDegrees <= SERVO_DEGREE_MAX; posDegrees += 5) {
      for(int servo = 0; servo < MAX_SERVOS; ++servo) {
        servoActionRequest( servo, posDegrees );
      }
    }

    for(long posDegrees = SERVO_DEGREE_MAX; posDegrees >= 0; posDegrees -= 90) {
      for(int servo = 0; servo < MAX_SERVOS; ++servo) {
        servoActionRequest( servo, posDegrees );
      }
    }
}

