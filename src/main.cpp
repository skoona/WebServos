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
#include <esp_wifi.h>
 
#include <Wire.h>               // Only needed for Arduino 1.6.5 and earlier
#include "SSD1306Wire.h"        // legacy: #include "SSD1306.h"

#include <FS.h>

#define CONFIG_FILENAME          F("/wifi_cred.dat")
#define PIN_LED   LED_BUILTIN
#define PIN_SDA           17  // blue
#define PIN_SCL           16  // gray

#define SKN_PGM_NAME      "WebServos"
#define SKN_PGM_VERSION   "v1.0.2"


#include <SPIFFS.h>
FS* filesystem =      &SPIFFS;
#define FileFS        SPIFFS
#define FS_Name       "SPIFFS"

// String ssid = "ESP_" + String((uint32_t)ESP.getEfuseMac()) + "-WebServos";
#define ESP_getChipId()   ((uint32_t)ESP.getEfuseMac())

//define your default values here, if there are different values in CALIBRATION_STANDARDS_FILE (config.json), they are overwritten.
#define CFG_PARAM_LEN                16


WifiTool wifiTool;
AsyncWebServer webServer(80);
AsyncWebSocket ws("/ws");
SSD1306Wire display(0x3c, PIN_SDA, PIN_SCL);  

#include "servos.h"

void toggleLED()
{
  //toggle state
  digitalWrite(PIN_LED, !digitalRead(PIN_LED));
}

void updateOLEDDisplay() {
  char buf[32];
  display.clear();
  display.setFont(ArialMT_Plain_16);
    display.drawString(0, 0, "IP: ");
    display.drawString(display.getStringWidth("IP: "), 0, WiFi.localIP().toString().c_str());
  
  display.setFont(ArialMT_Plain_10);
  for (int idx = 0, y = 15; idx < MAX_SERVOS; idx++, y += 10) {
    snprintf(buf, sizeof(buf), "Srv%d: %03u• %04u• %04uµv", idx, 
          calibrate[idx].degree, calibrate[idx].maxDegree, calibrate[idx].analog);
    display.drawString(0, y, buf);
  }

  display.display();
}

void initOledDisplay() {
  Wire.begin(PIN_SDA, PIN_SCL, 400000UL);
  display.init();

  display.flipScreenVertically();
  display.setFont(ArialMT_Plain_16);
  display.setTextAlignment(TEXT_ALIGN_LEFT);
  
  vTaskDelay(pdMS_TO_TICKS(1000));
  display.clear();

  // write the buffer to the display
  display.drawString(16, 0, SKN_PGM_NAME);
  display.drawString(32, 18, SKN_PGM_VERSION);

  display.setFont(ArialMT_Plain_10);
  display.drawString(16, 40, ARDUINO_BOARD);

  display.display();
}

// Callback: send 404 if requested file does not exist
void onPageNotFound(AsyncWebServerRequest *request) {
  IPAddress remote_ip = request->client()->remoteIP();
  Serial.println("[" + remote_ip.toString() +
                  "] HTTP GET request of " + request->url());
  request->send(404, "text/plain", "Not found");
}

void initWiFi() { 
  wifiTool.begin(false);
  if (!wifiTool.wifiAutoConnect()) {
    Serial.println("fail to connect to wifi!!!!");
    wifiTool.runApPortal();
  }
  updateOLEDDisplay();
  Serial.println("initWiFi() Exited!");
}

void initUI() {
  Serial.println("initUI(): Starting Primary Operations");
  
  webServer.serveStatic("/", SPIFFS, "/").setDefaultFile("servo.html").setFilter(ON_STA_FILTER);  
  webServer.onNotFound(onPageNotFound);

  // Start WebSocket server and assign callback
  ws.onEvent(onEvent);
  webServer.addHandler(&ws);
  webServer.begin();

  updateOLEDDisplay();

  Serial.println("initUI() Exited!");
}

void setup() {
  pinMode(LED_BUILTIN, OUTPUT);  

  Serial.begin(115200); 
  while (!Serial);

  initOledDisplay();

  Serial.print("\nStarting Async WebServos using " + String(FS_Name));
  Serial.println(" on " + String(ARDUINO_BOARD));

  // Enable FileFS 
  if (!FileFS.begin(false)) {
    Serial.print(FS_Name);
    Serial.println(F(" Required Service Offline!"));    
  }

  /* Handle WiFi/AP
  */
  initWiFi();

  /*
   * Init Servo
  */
  initServoControls();  

  /* Initilize UI
  */
  initUI();
  
  Serial.printf("Setup Complete !\n");
}

void loop() {
  servoRecordInterval(false);
  taskYIELD();
}
