/**
 * Project:     BasicOTA for VSCode-PlatformIO
 * Doc:         https://github.com/JakubAndrysek/BasicOTA-ESP32-library/blob/master/README.md
 * Proj URL:    https://github.com/JakubAndrysek/BasicOTA-ESP32-library
 * Author:      Kuba Andrýsek
 * Created:     2020-5-5
 * Website:     https://kubaandrysek.cz
 * Inspiration: https://lastminuteengineers.com/esp32-ota-updates-arduino-ide/
 * 
 * This src file is used to prepare the ESP to receive OTA updates. The SSID and PASSWORD are from the WiFi Network of CIT-116. For this step USB-serial connection is requiered to make a usual Upload. After uploading this file, the Serial monitor can be opened to verify the IP Address for the ESP32 in case it's not a static IP.
 */

#include <Arduino.h>
#include <WiFi.h>
#include "BasicOTA.hpp"

#define SSID "Robotat"
#define PASSWORD "iemtbmcit116"

BasicOTA OTA;

void setup()
{
  Serial.begin(115200);
  Serial.println("Startup");
  WiFi.mode(WIFI_STA);
  WiFi.begin(SSID, PASSWORD);
  while (WiFi.waitForConnectResult() != WL_CONNECTED)
  {
    Serial.println("Connection Failed! Rebooting...");
    delay(5000);
    ESP.restart();
  }

  OTA.begin(); // Setup settings

  Serial.println("Ready");
  Serial.print("IP address: ");
  Serial.println(WiFi.localIP());
}

void loop()
{
  OTA.handle();
}