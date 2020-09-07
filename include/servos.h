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

#include <ESPAsyncWebServer.h>
#include <ArduinoJson.h>          //https://github.com/bblanchon/ArduinoJson

#define MAX_SERVOS        4                 // Number of Servos for this Project

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

extern int giDegreeMax;   // Config Params
extern int giMaxRecCnt;   // Config Params 
extern int giMinPulseWidth;   // Config Params
extern int giMaxPulseWidth;   // Config Params 

extern AsyncWebSocket ws;

void initializeServoControls(); 
bool servoRecordInterval(bool startStopFlag);
void onEvent( AsyncWebSocket * server, AsyncWebSocketClient * client, AwsEventType type, void * arg, uint8_t *data, size_t len);

#endif /* !_SERVOS */
