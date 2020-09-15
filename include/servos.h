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
#define CONFIG_PARAM_LEN 16

#define CALIBRATION_STANDARDS_FILE   "/calibrate.json"
#define RECORDING_FILE               "/records.bin"
#define CALIBRATION_FILE             "/servos.bin"

typedef struct _qitem {
  uint32_t servo;
  uint32_t degree;
} QItem, *PQItem;

typedef struct __attribute__((packed)) _servoCalibration {
  char      name[CONFIG_PARAM_LEN];  // 16
  uint32_t  analog;
  uint32_t  degree;
  uint32_t  maxDegree;
  uint32_t  minPosition;
  uint32_t  maxPosition;
  uint32_t  minPulseWidth;
  uint32_t  maxPulseWidth;
}  ServoCalibration, *PServoCalibration;

extern ServoCalibration calibrate[MAX_SERVOS];
extern AsyncWebSocket ws;

void updateOLEDDisplay();
void toggleLED();
void initServoControls(); 
bool servoRecordInterval(bool startStopFlag);
void onEvent( AsyncWebSocket * server, AsyncWebSocketClient * client, AwsEventType type, void * arg, uint8_t *data, size_t len);

#endif /* !_SERVOS */
