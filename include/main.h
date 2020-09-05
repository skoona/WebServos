/** 
 *  File main.h  
 * 
*/

#ifndef _MAIN
#define _MAIN
#include <WebSocketsServer.h>

extern WebSocketsServer webSocket;
extern volatile unsigned long guiTimeBase,      // default time base, target 0.5 seconds
                  gulLastTimeBase,      // time base delta
                  gulRecordInterval,    // Ticks interval 0.5 seconds
                  gulRecordCounter;      // Number of recorded records

extern volatile bool   gbRecordMode,  // Recording
                  gvDuration,         // 1/2 second marker
                  gbWSOnline,         // WebSocket Client Connected
                  gbCalibrate,        // Servo Calibration Required
                  gbServoCalibrate;   // isCalibration Required


#endif /* !_SERVOS */
