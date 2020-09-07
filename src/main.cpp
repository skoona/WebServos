/** 
 * WebServos
 * 
 * Data Flow
 *  [Browser]        [ESP32]
 * currentState -->   send('state')
 * follow       -->   move Servo#
 * play         -->   noop
 * startRecord  -->   disable-move send('record') stream every interval
 * stopRecord   -->   enable-move  stop('record') stream
 * button[follow,play,startRecord,stopRecord] -> if no parms echo(button) only
*/ 


#include <wifiTool.h>
#include "servos.h"

WifiTool wifiTool;
AsyncWebServer sServer(80);
AsyncWebSocket ws("/ws");

// Callback: send 404 if requested file does not exist
void onPageNotFound(AsyncWebServerRequest *request) {
  IPAddress remote_ip = request->client()->remoteIP();
  Serial.println("[" + remote_ip.toString() +
                  "] HTTP GET request of " + request->url());
  request->send(404, "text/plain", "Not found");
}

void InitializeWiFi() {
  wifiTool.begin(false);
  if (!wifiTool.wifiAutoConnect()) {
    Serial.println("fail to connect to wifi!!!!");
    wifiTool.runApPortal();
  }
  Serial.println("InitializeWiFi() Exited!");
}

void initializeUI() {
  Serial.println("Starting Primary Operations");
  
  sServer.serveStatic("/", SPIFFS, "/").setDefaultFile("servo.html").setFilter(ON_STA_FILTER);  
  sServer.onNotFound(onPageNotFound);

  // Start WebSocket server and assign callback
  ws.onEvent(onEvent);
  sServer.addHandler(&ws);
  sServer.begin();

  Serial.println("initializeUI() Exited!");
}

void setup() {
  Serial.begin(115200); 
  Serial.println("Skoona.net Multple Servos with Feedback.");

  /* Handle WiFi/AP
  */
  InitializeWiFi();

  /*
   * Init Servo
  */
  initializeServoControls();  

  /* Initilize UI
  */
  initializeUI();
  
  Serial.printf("Setup Complete !\n");
}

void loop() {
  servoRecordInterval(false);

  taskYIELD();
}
