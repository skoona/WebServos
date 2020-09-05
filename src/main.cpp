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

#include "servos.h"
#include <wifiTool.h>
#include <WebSocketsServer.h>

volatile unsigned long guiTimeBase   = 0,      // default time base, target 0.5 seconds
                  gulLastTimeBase    = 0,      // time base delta
                  gulRecordInterval  = 500,    // Ticks interval 0.5 seconds
                  gulRecordCounter   = 0;      // Number of recorded records
volatile bool     gbRecordMode       = false,  // Recording
                  gvDuration         = false,  // 1/2 second marker
                  gbWSOnline         = false,  // WebSocket Client Connected
                  gbCalibrate        = true,   // Servo Calibration Required
                  gbServoCalibrate   = true;   // isCalibration Required

WifiTool wifiTool;
AsyncWebServer sServer(80);
WebSocketsServer webSocket = WebSocketsServer(1337);


// Callback: receiving any WebSocket message
void onWebSocketEvent(uint8_t client_num,
                      WStype_t type,
                      uint8_t * payload,
                      size_t length) {
  static char lastBuffer[128];        // filter duplicate messages
  char message[128];           // response messages

  DynamicJsonDocument doc(200);
  DeserializationError errorJson;

  // Figure out the type of WebSocket event
  switch(type) {

    // Client has disconnected
    case WStype_DISCONNECTED:
      gbWSOnline = false;
      Serial.printf("[%u] Disconnected!\n", client_num);
      break;

    // New client has connected
    case WStype_CONNECTED:
      {
        IPAddress ip = webSocket.remoteIP(client_num);
        Serial.printf("[%u] Connection from ", client_num);
        Serial.println(ip.toString());
        gbWSOnline = true;
      }
      break;

    // Handle text messages from client
    case WStype_TEXT:
      if ( strcmp((char *)payload, lastBuffer) == 0 && strcmp((char *)payload, "{\"action\":\"currentState\"}") != 0) { // duplicate message
        Serial.printf("[%u] Duplicate Message Received text: %s\n", client_num, payload);
        break;
      } else {
        strncpy(lastBuffer, (char *)payload, sizeof(lastBuffer));
      }

      // Print out raw message
      Serial.printf("[%u] Received text: %s\n", client_num, payload);
 
      errorJson = deserializeJson(doc, payload);
      if(errorJson) {
        Serial.printf("deserializeJson() failed! cause=%s\n", errorJson.c_str());
        break;
      }

      // determine action
      if ( strcmp(doc["action"], "currentState") == 0 ) { // move to neutral positon -- currentState
        Serial.printf("Current State Action \n");
        for(int i = 0; i < MAX_SERVOS; ++i) {
          snprintf(message, sizeof(message), "{\"action\":\"state\",\"servo\":%d,\"degree\":%d}", i, calibrate[i].degree);
          webSocket.broadcastTXT( message, strlen(message), false );
          Serial.printf("Sending currentState=%s\n", message);
        }

      } else if ( strcmp(doc["action"], "follow") == 0 ) {
        Serial.printf("Follow Action \n");
        if ( !gbRecordMode && doc.containsKey("degree") ) {
          servoActionRequest(doc["servo"], doc["degree"]);        
        } else if (!doc.containsKey("degree")) {
          attachServos();
          snprintf(message, sizeof(message), "{\"action\":\"follow\"}");
          Serial.printf("Follow: %s\n", message);
          webSocket.broadcastTXT( message, strlen(message), false );
        }

      } else if ( strcmp(doc["action"], "play") == 0 ) {
        Serial.printf("Play Action \n");
        if(gbRecordMode) {
          attachServos();
        }
        snprintf(message, sizeof(message), "{\"action\":\"play\"}");
        Serial.printf("Playing: %s\n", message);
        webSocket.broadcastTXT( message, strlen(message), false );

      } else if ( strcmp(doc["action"], "startRecord") == 0 ) {
        Serial.printf("StartRecord Action \n");
        if (!gbRecordMode) {
          detachServos();
        }
        snprintf(message, sizeof(message), "{\"action\":\"startRecord\"}");
        Serial.printf("Recording: %s\n", message);
        webSocket.broadcastTXT( message, strlen(message), false );

      } else if ( strcmp(doc["action"], "stopRecord") == 0 ) {
        Serial.printf("StopRecord Action \n");
        if(gbRecordMode) {
          attachServos();
        }
        snprintf(message, sizeof(message), "{\"action\":\"stopRecord\"}");
        Serial.printf("Stop Recording: %s\n", message);
        webSocket.broadcastTXT( message, strlen(message), false );

      } else {
        Serial.println("Message not recognized!");
      }
      break;

    case WStype_ERROR:
      Serial.printf("[%u] Error Received: %s\n", client_num, payload);
      break;

    default:
      break;
  }
}

// Callback: send homepage
void onIndexRequest(AsyncWebServerRequest *request) {
  IPAddress remote_ip = request->client()->remoteIP();
  Serial.println("[" + remote_ip.toString() +
                  "] HTTP GET request of " + request->url());
  request->send(SPIFFS, "/servo.html", "text/html");
}
void onCSSRequest(AsyncWebServerRequest *request) {
  IPAddress remote_ip = request->client()->remoteIP();
  Serial.println("[" + remote_ip.toString() +
                  "] HTTP GET request of " + request->url());
  request->send(SPIFFS, "/mini-default.css", "text/css");
}
void onJS2Request(AsyncWebServerRequest *request) {
  IPAddress remote_ip = request->client()->remoteIP();
  Serial.println("[" + remote_ip.toString() +
                  "] HTTP GET request of " + request->url());
  request->send(SPIFFS, "/jquery.min.js", "text/javascript");
}
void onJSRequest(AsyncWebServerRequest *request) {
  IPAddress remote_ip = request->client()->remoteIP();
  Serial.println("[" + remote_ip.toString() +
                  "] HTTP GET request of " + request->url());
  request->send(SPIFFS, "/jogDial.min.js", "text/javascript");
}
void onImgKnobRequest(AsyncWebServerRequest *request) {
  IPAddress remote_ip = request->client()->remoteIP();
  Serial.println("[" + remote_ip.toString() +
                  "] HTTP GET request of " + request->url());
  request->send(SPIFFS, "/base_one_knob.png", "image/png");
}
void onImgBgRequest(AsyncWebServerRequest *request) {
  IPAddress remote_ip = request->client()->remoteIP();
  Serial.println("[" + remote_ip.toString() +
                  "] HTTP GET request of " + request->url());
  request->send(SPIFFS, "/base_one_bg.png", "image/png");
}
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

void initializeUI(){
  Serial.println("Starting Primary Operations");
  // On HTTP request for root, provide index.html file
  // sServer.reset(); // begin();
  sServer.serveStatic("/", SPIFFS, "/").setDefaultFile("servo.html").setFilter(ON_STA_FILTER);  
  sServer.on("/servo", HTTP_GET, onIndexRequest);
  sServer.on("/jquery.min.js", HTTP_GET, onJS2Request);
  sServer.on("/mini-default.css", HTTP_GET, onCSSRequest);
  sServer.on("/jogDial.min.js", HTTP_GET, onJSRequest);
  sServer.on("/base_one_knob.png", HTTP_GET, onImgKnobRequest);
  sServer.on("/base_one_bg.png", HTTP_GET, onImgBgRequest);
  sServer.onNotFound(onPageNotFound);
  sServer.begin();

  // Start WebSocket server and assign callback
  webSocket.begin();
  webSocket.onEvent(onWebSocketEvent);
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
  guiTimeBase = millis();
  gvDuration = ((guiTimeBase - gulLastTimeBase) >= gulRecordInterval);

  if (gbWSOnline && gvDuration && gbRecordMode) {
    servoRecordRequest();
  }

  if (gvDuration) { // Every half second poll 
    gulLastTimeBase = guiTimeBase;    
    webSocket.loop();
  }

  taskYIELD();
}
