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
#define PARAMS_CONFIG_FILENAME   F("/params.json")
#define PIN_LED LED_BUILTIN
#define PIN_SDA           17  // blue
#define PIN_SCL           16  // gray

#include <SPIFFS.h>
FS* filesystem =      &SPIFFS;
#define FileFS        SPIFFS
#define FS_Name       "SPIFFS"

// String ssid = "ESP_" + String((uint32_t)ESP.getEfuseMac()) + "-WebServos";
#define ESP_getChipId()   ((uint32_t)ESP.getEfuseMac())

//define your default values here, if there are different values in PARAMS_CONFIG_FILENAME (config.json), they are overwritten.
#define CFG_PARAM_LEN                16
char cfgDegreeMax[CFG_PARAM_LEN] = "270";
char cfgMaxRecCnt[CFG_PARAM_LEN] = "240";
int  giDegreeMax = 270;
int  giMaxRecCnt = 240;

char cfgMinPulse[CFG_PARAM_LEN] = "500";
char cfgMaxPulse[CFG_PARAM_LEN] = "2500";
int  giMinPulseWidth = 500;
int  giMaxPulseWidth = 2500;


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
  display.clear();
  display.display();
  display.setFont(ArialMT_Plain_16);
    display.drawString(0, 0, "IP: ");
    display.drawString(display.getStringWidth("IP: "), 0, WiFi.localIP().toString().c_str());

  // display.setFont(ArialMT_Plain_10);
    display.drawString(0, 16, "DegreeMax");
    display.drawString(display.getStringWidth("DegreeMax: "), 16, cfgDegreeMax);
    display.drawString(0, 32, "MaxRecCnt" );
    display.drawString(display.getStringWidth("MaxRecCnt: "), 32, cfgMaxRecCnt );
    display.drawString(0, 48, "MinPulse" );
    display.drawString(display.getStringWidth("DegreeMax: "), 48, cfgMinPulse );
    display.drawString(0, 64, "MaxPulse" );
    display.drawString(display.getStringWidth("DegreeMax: "), 64, cfgMaxPulse );

  display.display();
}

bool loadFileFSConfigFile(void) {
  //read configuration from FS json
  Serial.println("Mounting FS...");

  if (FileFS.begin())
  {
    Serial.println("Mounted file system");

    if (FileFS.exists(PARAMS_CONFIG_FILENAME))
    {
      //file exists, reading and loading
      Serial.println("Reading config file");
      File configFile = FileFS.open(PARAMS_CONFIG_FILENAME, FILE_READ);

      if (configFile)
      {
        Serial.print("Opened config file, size = ");
        size_t configFileSize = configFile.size();
        Serial.println(configFileSize);

        // Allocate a buffer to store contents of the file.
        std::unique_ptr<char[]> buf(new char[configFileSize + 1]);

        configFile.readBytes(buf.get(), configFileSize);

        Serial.print("\nJSON parseObject() result : ");

        DynamicJsonDocument json(1024);
        auto deserializeError = deserializeJson(json, buf.get(), configFileSize);
        if ( deserializeError )
        {
          Serial.println("failed");
          configFile.close();
          return false;
        }
        else
        {
          Serial.println("OK");

          if (json["cfgDegreeMax"]) {
            strncpy(cfgDegreeMax, json["cfgDegreeMax"], sizeof(cfgDegreeMax));
            giDegreeMax = atoi(json["cfgDegreeMax"]);          
          }
         
          if (json["cfgMaxRecCnt"]) {
            strncpy(cfgMaxRecCnt, json["cfgMaxRecCnt"], sizeof(cfgMaxRecCnt)); 
            giMaxRecCnt = atoi(json["cfgMaxRecCnt"]);
          }

          if (json["cfgMinPulse"]) {
            strncpy(cfgMinPulse, json["cfgMinPulse"], sizeof(cfgMinPulse)); 
            giMinPulseWidth = atoi(json["cfgMinPulse"]);
          }

          if (json["cfgMaxPulse"]) {
            strncpy(cfgMaxPulse, json["cfgMaxPulse"], sizeof(cfgMaxPulse)); 
            giMaxPulseWidth = atoi(json["cfgMaxPulse"]);
          }
        }

        serializeJsonPretty(json, Serial);

        configFile.close();
      }
    }
  }
  else
  {
    Serial.println("failed to mount FS");
    return false;
  }
  return true;
}

bool saveFileFSConfigFile(void)
{
  Serial.println("Saving config");

  DynamicJsonDocument json(128);

  // use this callback to update display
  updateOLEDDisplay();

  itoa( giDegreeMax, cfgDegreeMax, 10);
  itoa( giMaxRecCnt, cfgMaxRecCnt, 10);
  itoa( giMinPulseWidth, cfgMinPulse, 10);
  itoa( giMaxPulseWidth, cfgMaxPulse, 10);

  json["cfgDegreeMax"] = cfgDegreeMax;
  json["cfgMaxRecCnt"] = cfgMaxRecCnt;
  json["cfgMinPulse"] = cfgMinPulse;
  json["cfgMaxPulse"] = cfgMaxPulse;

  File configFile = FileFS.open(PARAMS_CONFIG_FILENAME, FILE_WRITE);  // FILE_APPEND

  if (!configFile)
  {
    Serial.println("Failed to open config file for writing");
    return false;
  }

  //serializeJson(json, Serial);
  serializeJsonPretty(json, Serial);
  // Write data to file and close it
  serializeJson(json, configFile);

  configFile.close();
  //end save
  return true;
}


// Callback: send 404 if requested file does not exist
void onPageNotFound(AsyncWebServerRequest *request) {
  IPAddress remote_ip = request->client()->remoteIP();
  Serial.println("[" + remote_ip.toString() +
                  "] HTTP GET request of " + request->url());
  request->send(404, "text/plain", "Not found");
}

void initializeWiFi() {
  loadFileFSConfigFile();
  
  wifiTool.begin(false);
  if (!wifiTool.wifiAutoConnect()) {
    Serial.println("fail to connect to wifi!!!!");
    wifiTool.runApPortal();
  }

  saveFileFSConfigFile();
  
  Serial.println("InitializeWiFi() Exited!");
}

void initializeUI() {
  Serial.println("Starting Primary Operations");
  
  webServer.serveStatic("/", SPIFFS, "/").setDefaultFile("servo.html").setFilter(ON_STA_FILTER);  
  webServer.onNotFound(onPageNotFound);

  // Start WebSocket server and assign callback
  ws.onEvent(onEvent);
  webServer.addHandler(&ws);
  webServer.begin();

  Serial.println("initializeUI() Exited!");
}

void setup() {
  pinMode(LED_BUILTIN, OUTPUT);  

  Serial.begin(115200); 
  while (!Serial);

  Wire.begin(PIN_SDA, PIN_SCL, 400000U);
  display.init();

  display.flipScreenVertically();
  display.setFont(ArialMT_Plain_10);
  display.setTextAlignment(TEXT_ALIGN_LEFT);
  display.clear();

  // write the buffer to the display
  display.drawString(0, 0, "WebServos");
  display.display();
  
  Serial.print("\nStarting Async WebServos using " + String(FS_Name));
  Serial.println(" on " + String(ARDUINO_BOARD));

  // Format FileFS if not yet
  if (!FileFS.begin(false)) {
    Serial.print(FS_Name);
    Serial.println(F(" Required Service Offline!"));    
  }
  /* Handle WiFi/AP
  */
  initializeWiFi();

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
