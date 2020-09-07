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

/****************************************************************************************************************************
 Minimal Config
 *****************************************************************************************************************************/
#if !( defined(ESP32) )
  #error This code is intended to run on the ESP32 platform! Please check your Tools->Board setting.
#endif

// Use from 0 to 4. Higher number, more debugging messages and memory usage.
#define _ESPASYNC_WIFIMGR_LOGLEVEL_    4

#include <FS.h>
  
//Ported to ESP32
#include <esp_wifi.h>
#include <WiFi.h>
#include <WiFiClient.h>
#include <WiFiMulti.h>
WiFiMulti wifiMulti;

#define USE_SPIFFS      true
#include <SPIFFS.h>
FS* filesystem =      &SPIFFS;
#define FileFS        SPIFFS
#define FS_Name       "SPIFFS"

#define ESP_getChipId()   ((uint32_t)ESP.getEfuseMac())

#define LED_BUILTIN       22
#define LED_ON            HIGH
#define LED_OFF           LOW


// Pin D2 mapped to pin GPIO2/ADC12 of ESP32, or GPIO2/TXD1 of NodeMCU control on-board LED
#define PIN_LED       LED_BUILTIN

// Now support ArduinoJson 6.0.0+ ( tested with v6.14.1 )
#include <ArduinoJson.h>          //https://github.com/bblanchon/ArduinoJson

char configFileName[] = "/config.json";

// SSID and PW for your Router
String Router_SSID;
String Router_Pass;

#define FORMAT_FILESYSTEM      false
#define MIN_AP_PASSWORD_SIZE    8
#define SSID_MAX_LEN            32
#define PASS_MAX_LEN            64
#define NUM_WIFI_CREDENTIALS     2
#define CONFIG_FILENAME          F("/wifi_cred.dat")

typedef struct
{
  char wifi_ssid[SSID_MAX_LEN];
  char wifi_pw  [PASS_MAX_LEN];
}  WiFi_Credentials;

typedef struct
{
  String wifi_ssid;
  String wifi_pw;
}  WiFi_Credentials_String;

typedef struct
{
  WiFi_Credentials  WiFi_Creds [NUM_WIFI_CREDENTIALS];
} WM_Config;

WM_Config         WM_config;

// Indicates whether ESP has WiFi credentials saved from previous session, or double reset detected
bool initialConfig = false;

// SSID and PW for Config Portal
String AP_SSID = "ESP32_DevKit";
String AP_PASS = "Password";

#define USE_AVAILABLE_PAGES         true
#define USE_ESP_WIFIMANAGER_NTP     false
#define USE_CLOUDFLARE_NTP          false
#define USING_CORS_FEATURE          false

#define USE_STATIC_IP_CONFIG_IN_CP  false
#define USE_DHCP_IP     true

IPAddress stationIP   = IPAddress(0, 0, 0, 0);
IPAddress gatewayIP   = IPAddress(192, 168, 4, 1);
IPAddress netMask     = IPAddress(255, 255, 255, 0);

#define USE_CONFIGURABLE_DNS      true
IPAddress dns1IP      = gatewayIP;
IPAddress dns2IP      = IPAddress(8, 8, 8, 8);          // Google's

#include <ESPAsync_WiFiManager.h>              //https://github.com/khoih-prog/ESPAsync_WiFiManager
#define HTTP_PORT           80

AsyncWebServer webServer(HTTP_PORT);
AsyncWebSocket ws("/ws");
DNSServer dnsServer;

#include "servos.h"


//define your default values here, if there are different values in configFileName (config.json), they are overwritten.
#define CFG_PARAM_LEN                16
char cfgDegreeMax[CFG_PARAM_LEN] = "270";
char cfgMaxRecCnt[CFG_PARAM_LEN] = "240";
int  giDegreeMax = 270;
int  giMaxRecCnt = 240;

char cfgMinPulse[CFG_PARAM_LEN] = "500";
char cfgMaxPulse[CFG_PARAM_LEN] = "2500";
int  giMinPulseWidth = 500;
int  giMaxPulseWidth = 2500;


//flag for saving data
bool shouldSaveConfig = false;

//callback notifying us of the need to save config
void saveConfigCallback(void) {
  Serial.println("Should save config");
  shouldSaveConfig = true;
}

bool loadFileFSConfigFile(void) {
  //read configuration from FS json
  Serial.println("Mounting FS...");

  if (FileFS.begin())
  {
    Serial.println("Mounted file system");

    if (FileFS.exists(configFileName))
    {
      //file exists, reading and loading
      Serial.println("Reading config file");
      File configFile = FileFS.open(configFileName, "r");

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


  json["cfgDegreeMax"] = String( giDegreeMax ).c_str();
  json["cfgMaxRecCnt"] = String( giMaxRecCnt ).c_str();
  json["cfgMinPulse"] = String( giMinPulseWidth ).c_str();
  json["cfgMaxPulse"] = String( giMaxPulseWidth ).c_str();

  File configFile = FileFS.open(configFileName, FILE_WRITE);  // FILE_APPEND

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

uint8_t connectMultiWiFi(void)
{
  // For ESP32, this better be 0 to shorten the connect time
  #define WIFI_MULTI_1ST_CONNECT_WAITING_MS       0
  #define WIFI_MULTI_CONNECT_WAITING_MS           100L
  
  uint8_t status;

  LOGERROR(F("ConnectMultiWiFi with :"));
  
  if ( (Router_SSID != "") && (Router_Pass != "") )
  {
    LOGERROR3(F("* Flash-stored Router_SSID = "), Router_SSID, F(", Router_Pass = "), Router_Pass );
  }

  for (uint8_t i = 0; i < NUM_WIFI_CREDENTIALS; i++)
  {
    // Don't permit NULL SSID and password len < MIN_AP_PASSWORD_SIZE (8)
    if ( (String(WM_config.WiFi_Creds[i].wifi_ssid) != "") && (strlen(WM_config.WiFi_Creds[i].wifi_pw) >= MIN_AP_PASSWORD_SIZE) )
    {
      LOGERROR3(F("* Additional SSID = "), WM_config.WiFi_Creds[i].wifi_ssid, F(", PW = "), WM_config.WiFi_Creds[i].wifi_pw );
    }
  }
  
  LOGERROR(F("Connecting MultiWifi..."));

  WiFi.mode(WIFI_STA);

  int i = 0;
  status = wifiMulti.run();
  delay(WIFI_MULTI_1ST_CONNECT_WAITING_MS);

  while ( ( i++ < 20 ) && ( status != WL_CONNECTED ) )
  {
    status = wifiMulti.run();

    if ( status == WL_CONNECTED )
      break;
    else
      delay(WIFI_MULTI_CONNECT_WAITING_MS);
  }

  if ( status == WL_CONNECTED )
  {
    LOGERROR1(F("WiFi connected after time: "), i);
    LOGERROR3(F("SSID:"), WiFi.SSID(), F(",RSSI="), WiFi.RSSI());
    LOGERROR3(F("Channel:"), WiFi.channel(), F(",IP address:"), WiFi.localIP() );
  }
  else
    LOGERROR(F("WiFi not connected"));

  return status;
}

void toggleLED()
{
  //toggle state
  digitalWrite(PIN_LED, !digitalRead(PIN_LED));
}

void check_WiFi(void)
{
  if ( (WiFi.status() != WL_CONNECTED) )
  {
    Serial.println(F("\nWiFi lost. Call connectMultiWiFi in loop"));
    connectMultiWiFi();
  }
}  

void check_status(void)
{
  static ulong LEDstatus_timeout    = 0;
  static ulong checkwifi_timeout    = 0;

  static ulong current_millis;

#define WIFICHECK_INTERVAL    10000L
#define LED_INTERVAL           2000L

  current_millis = millis();
  
  // Check WiFi every WIFICHECK_INTERVAL (1) seconds.
  if ((current_millis > checkwifi_timeout) || (checkwifi_timeout == 0))
  {
    check_WiFi();
    checkwifi_timeout = current_millis + WIFICHECK_INTERVAL;
  }

  if ((current_millis > LEDstatus_timeout) || (LEDstatus_timeout == 0))
  {
    // Toggle LED at LED_INTERVAL = 2s
    toggleLED();
    LEDstatus_timeout = current_millis + LED_INTERVAL;
  }
}

void loadConfigData(void)
{
  File file = FileFS.open(CONFIG_FILENAME, "r");
  LOGERROR(F("LoadWiFiCfgFile "));

  if (file)
  {
    file.readBytes((char *) &WM_config, sizeof(WM_config));
    file.close();
    LOGERROR(F("OK"));
  }
  else
  {
    LOGERROR(F("failed"));
  }
}
    
void saveConfigData(void)
{
  File file = FileFS.open(CONFIG_FILENAME, "w");
  LOGERROR(F("SaveWiFiCfgFile "));

  if (file)
  {
    file.write((uint8_t*) &WM_config, sizeof(WM_config));
    file.close();
    LOGERROR(F("OK"));
  }
  else
  {
    LOGERROR(F("failed"));
  }
}

// Callback: send 404 if requested file does not exist
void onPageNotFound(AsyncWebServerRequest *request) {
  IPAddress remote_ip = request->client()->remoteIP();
  Serial.println("[" + remote_ip.toString() +
                  "] HTTP GET request of " + request->url());
  request->send(404, "text/plain", "Not found");
}


void InitializeWiFi() {

  loadFileFSConfigFile();

  // The extra parameters to be configured (can be either global or just in the setup)
  // After connecting, parameter.getValue() will get you the configured value
  // id/name placeholder/prompt default length
  ESPAsync_WMParameter cfgServoDegreeMaxParam("cfgDegreeMax", "Max Servo Degree", cfgDegreeMax, CFG_PARAM_LEN + 1);
  ESPAsync_WMParameter cfgServoMaxRecordCountParam  ("cfgMaxRecCnt",   "Max Servo Recordings",   cfgMaxRecCnt,   CFG_PARAM_LEN + 1);
  ESPAsync_WMParameter cfgServoMinPulseWidthParam  ("cfgMinPulse",   "Min Servo Pulse(µs)",   cfgMinPulse,   CFG_PARAM_LEN + 1);
  ESPAsync_WMParameter cfgServoMaxPulseWidthParam  ("cfgMaxPulse",   "Max Servo Pulse(µs)",   cfgMaxPulse,   CFG_PARAM_LEN + 1);

  unsigned long startedAt = millis();

  //Local intialization. Once its business is done, there is no need to keep it around
  // Use this to default DHCP hostname to ESP8266-XXXXXX or ESP32-XXXXXX
  //ESPAsync_WiFiManager ESPAsync_wifiManager(&webServer, &dnsServer);
  // Use this to personalize DHCP hostname (RFC952 conformed)
  ESPAsync_WiFiManager ESPAsync_wifiManager(&webServer, &dnsServer, "WebServos");

  //set config save notify callback
  ESPAsync_wifiManager.setSaveConfigCallback(saveConfigCallback);

  //add all your parameters here
  ESPAsync_wifiManager.addParameter(&cfgServoDegreeMaxParam);
  ESPAsync_wifiManager.addParameter(&cfgServoMaxRecordCountParam);
  ESPAsync_wifiManager.addParameter(&cfgServoMinPulseWidthParam);
  ESPAsync_wifiManager.addParameter(&cfgServoMaxPulseWidthParam);

  ESPAsync_wifiManager.setDebugOutput(true);

  //set custom ip for portal
  ESPAsync_wifiManager.setAPStaticIPConfig(IPAddress(192, 168, 4, 1), IPAddress(192, 168, 4, 1), IPAddress(255, 255, 255, 0));

  //set minimum quality of signal so it ignores AP's under that quality
  //defaults to 8%
  //ESPAsync_wifiManager.setMinimumSignalQuality();
  ESPAsync_wifiManager.setMinimumSignalQuality(-1);

  // From v1.0.10 only
  // Set config portal channel, default = 1. Use 0 => random channel from 1-13
  ESPAsync_wifiManager.setConfigPortalChannel(0);
  //////
  
  // We can't use WiFi.SSID() in ESP32 as it's only valid after connected.
  // SSID and Password stored in ESP32 wifi_ap_record_t and wifi_config_t are also cleared in reboot
  // Have to create a new function to store in EEPROM/SPIFFS/LittleFS for this purpose
  Router_SSID = ESPAsync_wifiManager.WiFi_SSID();
  Router_Pass = ESPAsync_wifiManager.WiFi_Pass();

  //Remove this line if you do not want to see WiFi password printed
  Serial.printf("\nStored: SSID = %s, Pass = %s\n", Router_SSID.c_str(), Router_Pass.c_str() );

  if ( Router_SSID != "" ) // || Router_SSID.isEmpty() )  WIFI_MODE_NULL == 0
  {
    ESPAsync_wifiManager.setConfigPortalTimeout(120); //If no access point name has been previously entered disable timeout.
    Serial.println("Got stored Credentials. Timeout 120s");
  }
  else
  {
    Serial.println("No stored Credentials. No timeout");
  }

  String chipID = String(ESP_getChipId(), HEX);
  chipID.toUpperCase();

  // SSID and PW for Config Portal
  AP_SSID = "ESP_" + chipID + "_ServoAP";
  AP_PASS = "ESP_" + chipID;

  // From v1.1.0, Don't permit NULL password
  if ( (Router_SSID == "") || (Router_Pass == "") || ( Router_SSID.isEmpty() ) )
  {
    Serial.println("We haven't got any access point credentials, so get them now");

    initialConfig = true;

    // Starts an access point
    if ( !ESPAsync_wifiManager.startConfigPortal(AP_SSID.c_str(), AP_PASS.c_str()) )
      Serial.println("Not connected to WiFi but continuing anyway.");
    else
      Serial.println("WiFi connected... :)");

    // Stored  for later usage, from v1.1.0, but clear first
    memset(&WM_config, 0, sizeof(WM_config));
    
    for (uint8_t i = 0; i < NUM_WIFI_CREDENTIALS; i++)
    {
      String tempSSID = ESPAsync_wifiManager.getSSID(i);
      String tempPW   = ESPAsync_wifiManager.getPW(i);
  
      if (strlen(tempSSID.c_str()) < sizeof(WM_config.WiFi_Creds[i].wifi_ssid) - 1)
        strcpy(WM_config.WiFi_Creds[i].wifi_ssid, tempSSID.c_str());
      else
        strncpy(WM_config.WiFi_Creds[i].wifi_ssid, tempSSID.c_str(), sizeof(WM_config.WiFi_Creds[i].wifi_ssid) - 1);

      if (strlen(tempPW.c_str()) < sizeof(WM_config.WiFi_Creds[i].wifi_pw) - 1)
        strcpy(WM_config.WiFi_Creds[i].wifi_pw, tempPW.c_str());
      else
        strncpy(WM_config.WiFi_Creds[i].wifi_pw, tempPW.c_str(), sizeof(WM_config.WiFi_Creds[i].wifi_pw) - 1);  

      // Don't permit NULL SSID and password len < MIN_AP_PASSWORD_SIZE (8)
      if ( (String(WM_config.WiFi_Creds[i].wifi_ssid) != "") && (strlen(WM_config.WiFi_Creds[i].wifi_pw) >= MIN_AP_PASSWORD_SIZE) )
      {
        LOGERROR3(F("* Add SSID = "), WM_config.WiFi_Creds[i].wifi_ssid, F(", PW = "), WM_config.WiFi_Creds[i].wifi_pw );
        wifiMulti.addAP(WM_config.WiFi_Creds[i].wifi_ssid, WM_config.WiFi_Creds[i].wifi_pw);
      }
    }

    saveConfigData();
  }
  else
  {
    wifiMulti.addAP(Router_SSID.c_str(), Router_Pass.c_str());
  }

  startedAt = millis();

  if (!initialConfig)
  {
    // Load stored data, the addAP ready for MultiWiFi reconnection
    loadConfigData();

    for (uint8_t i = 0; i < NUM_WIFI_CREDENTIALS; i++)
    {
      // Don't permit NULL SSID and password len < MIN_AP_PASSWORD_SIZE (8)
      if ( (String(WM_config.WiFi_Creds[i].wifi_ssid) != "") && (strlen(WM_config.WiFi_Creds[i].wifi_pw) >= MIN_AP_PASSWORD_SIZE) )
      {
        LOGERROR3(F("* Add SSID = "), WM_config.WiFi_Creds[i].wifi_ssid, F(", PW = "), WM_config.WiFi_Creds[i].wifi_pw );
        wifiMulti.addAP(WM_config.WiFi_Creds[i].wifi_ssid, WM_config.WiFi_Creds[i].wifi_pw);
      }
    }

    if ( WiFi.status() != WL_CONNECTED ) 
    {
      Serial.println("ConnectMultiWiFi in setup");
     
      connectMultiWiFi();
    }
  }

  Serial.print("After waiting ");
  Serial.print((float) (millis() - startedAt) / 1000L);
  Serial.print(" secs more in setup(), connection result is ");

  if (WiFi.status() == WL_CONNECTED)
  {
    Serial.print("connected. Local IP: ");
    Serial.println(WiFi.localIP());
  }
  else
    Serial.println(ESPAsync_wifiManager.getStatus(WiFi.status()));

  //read updated parameters
  strncpy(cfgDegreeMax, cfgServoDegreeMaxParam.getValue(), sizeof(cfgDegreeMax));
  strncpy(cfgMaxRecCnt,   cfgServoMaxRecordCountParam.getValue(),   sizeof(cfgMaxRecCnt));
  strncpy(cfgMinPulse, cfgServoMinPulseWidthParam.getValue(), sizeof(cfgMinPulse));
  strncpy(cfgMaxPulse,   cfgServoMaxPulseWidthParam.getValue(),   sizeof(cfgMaxPulse));

  //save the custom parameters to FS
  if (shouldSaveConfig)
  {
    saveFileFSConfigFile();
  }

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
  
  Serial.print("Local IP = ");
  Serial.println(WiFi.localIP());

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
  check_status();
  servoRecordInterval(false);
  taskYIELD();
}
