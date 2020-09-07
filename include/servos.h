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
#define SERVO_PULSE_MIN 500                 // Min Servo Pulse Width
#define SERVO_PULSE_MAX 2500                // Max Servo Pulse width
#define MAX_REQUEST_QUEUE_SZ 8              // Number of Request that can be queued


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

extern volatile bool gbRecordMode,  // Recording
                     gbWSOnline;
extern int servoDegreeMax;     // Config Params
extern int servoRecordMax;     // Config Params

extern AsyncWebSocket ws;

void initializeServoControls(); 
JsonObject readRecordedMovements(String jsonFilePathname);
bool saveRecordedMovements(const uint8_t *payload, String jsonFilePathname);
bool servoRecordInterval(bool startStopFlag);
void onEvent( AsyncWebSocket * server, AsyncWebSocketClient * client, AwsEventType type, void * arg, uint8_t *data, size_t len);

#endif /* !_SERVOS */
