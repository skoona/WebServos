# WebServos
ESP32 Controlling analog-feedback servos.
ESP32 PlatformIO based project

Features:
* Control Multiple Servos
* Follow Dial position
* Record Phyical movement of Servos @ 0.5ms interval
* Replay last recorded sequence
* When no saved wiFi credentials presents 
* * Standalone AP Configuration 
* * APnm: `ESP32_<hex-chip-id>_ServoAP` 
* * APpw: `ESP32_<hex-chip-id>`

# User Interface
![ScreenShot](./UI.png)

## ESP32 Board
LOLIN32 V1.0.0 WiFi + bluetooth Module ESP-32 4MB FLASH Development Board

## Key Libraries

* [ESPAsync_WiFiManager](https://github.com/khoih-prog/ESPAsync_WiFiManager)
* [ESPAsyncWebServer](https://github.com/me-no-dev/ESPAsyncWebServer)
* [ServoESP32](https://github.com/RoboticsBrno/ServoESP32)
* [ESP32AnalogRead](https://github.com/madhephaestus/ESP32AnalogRead)

Library Storage: ./PlatformIO/Projects/WebServos/.pio/libdeps/lolin32

    Updating bblanchon/ArduinoJson                6.16.1          [Up-to-date]
    Updating roboticsbrno/ServoESP32              1.0.3           [Up-to-date]
    Updating madhephaestus/ESP32AnalogRead        0.0.5           [Up-to-date]
    Updating me-no-dev/AsyncTCP                   1.1.1           [Up-to-date]
    Updating me-no-dev/ESP Async WebServer        1.2.3           [Up-to-date]
    Updating git+https://github.com/oferzv/wifiTool.git#master    [Up-to-date]

## License

    Open source under the MIT License.