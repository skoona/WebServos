/** 
 *  File servos.h  
 * 
 * 
 *       Servo-->    0    1    2    3
 * PWM Output Pin   21   19   23   18 
 * PWM Channel       4    5    6    7
 * ADC Input Pin    32   33   34   35
 * 
 */
#ifndef _SERVOS
#define _SERVOS

#include <Arduino.h>
#include <FS.h>
#include <SPIFFS.h>
#include <Servo.h>
#include <ESP32AnalogRead.h>
#include <Preferences.h>
#include <ArduinoJson.h>          //https://github.com/bblanchon/ArduinoJson

#define MAX_SERVOS        4                 // Number of Servos for this Project
#define SERVO_PULSE_MIN 500                 // Min Servo Pulse Width
#define SERVO_PULSE_MAX 2500                // Max Servo Pulse width
#define MAX_REQUEST_QUEUE_SZ 32             // Number of Request that can be queued

static const int       servosPins[MAX_SERVOS] = {21, 19, 23, 18};
static const int   servosPosition[MAX_SERVOS] = {32, 33, 34, 35};
static const int  servoMtrChannel[MAX_SERVOS] = {0, 1, 2, 3};
static const int servoFdBKChannel[MAX_SERVOS] = {4, 5, 6, 7};
static int servoDegreeMax = 270;
static int servoRecordMax = 240;
#define SERVO_DEGREE_MAX servoDegreeMax
#define MAX_RECORD_COUNT servoRecordMax         // sequence, servo, degree

extern QueueHandle_t qRequest[MAX_SERVOS];

typedef struct _qitem {
  uint32_t servo;
  uint32_t degree;
  uint32_t timeSequence;
} QItem, *PQItem;

typedef struct __attribute__((packed)) _servoCalibration {
  uint32_t  current;
  uint32_t  degree;
  uint32_t  minPosition;
  uint32_t  maxPosition;
}  ServoCalibration, *PServoCalibration;

extern ServoCalibration calibrate[MAX_SERVOS];


void calibrateServos(long degrees);
uint32_t readServoAnalog(int servo);
bool attachServos();
bool detachServos();
void servoRecordRequest();
void initializeServoControls(); 
void initializeServoTasks();
void servoActionRequest(int servo, int degrees);
void servoTask(void * pServo);
JsonObject readRecordedMovements(String jsonFilePathname);
bool saveRecordedMovements(const uint8_t *payload, String jsonFilePathname);


#endif /* !_SERVOS */
