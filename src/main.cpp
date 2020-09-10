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

#include <FS.h>

#define CONFIG_FILENAME          F("/wifi_cred.dat")
#define PARAMS_CONFIG_FILENAME   F("/params.json")
#define PIN_LED LED_BUILTIN

#include <SPIFFS.h>
FS* filesystem =      &SPIFFS;
#define FileFS        SPIFFS
#define FS_Name       "SPIFFS"

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

#include "servos.h"

void toggleLED()
{
  //toggle state
  digitalWrite(PIN_LED, !digitalRead(PIN_LED));
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
            strncpy(cfgMaxRecCnt, json["cfgMinPulse"], sizeof(cfgMinPulse)); 
            giMinPulseWidth = atoi(json["cfgMinPulse"]);
          }

          if (json["cfgMaxPulse"]) {
            strncpy(cfgMaxRecCnt, json["cfgMaxPulse"], sizeof(cfgMaxPulse)); 
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
