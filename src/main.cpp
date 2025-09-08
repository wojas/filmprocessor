#include <Arduino.h>
#include <WiFi.h>
#include <ArduinoOTA.h>
#include "secrets.h"

#define LED 2

const char *ssid = SECRET_WIFI_SSID;
const char *password = SECRET_WIFI_PASS;
uint32_t last_ota_time = 0;

void setup() {
  Serial.begin(115200);
  pinMode(LED,OUTPUT);

  // https://community.platformio.org/t/esp32-ota-using-platformio/15057/4
  Serial.print("[WiFi] Connecting to WiFI network ");
  Serial.println(ssid);
  digitalWrite(LED,HIGH);
  WiFi.mode(WIFI_STA);
  WiFi.begin(ssid, password);
  while (WiFi.waitForConnectResult() != WL_CONNECTED) {
    digitalWrite(LED,LOW);
    Serial.println("[WiFi] Connection Failed! Rebooting...");
    delay(5000);
    ESP.restart();
  }
  Serial.println("[WiFI] connected");
  Serial.print("[WiFi] IP address: ");
  Serial.println(WiFi.localIP());


  // https://github.com/espressif/arduino-esp32/blob/master/libraries/ArduinoOTA/examples/BasicOTA/BasicOTA.ino
  ArduinoOTA.setPassword("thaal6aiJievee");
  ArduinoOTA
    .onStart([]() {
      String type;
      if (ArduinoOTA.getCommand() == U_FLASH) {
        type = "sketch";
      } else {  // U_SPIFFS
        type = "filesystem";
      }

      // NOTE: if updating SPIFFS this would be the place to unmount SPIFFS using SPIFFS.end()
      Serial.println("Start updating " + type);
    })
    .onEnd([]() {
      Serial.println("\nEnd");
    })
    .onProgress([](unsigned int progress, unsigned int total) {
      if (millis() - last_ota_time > 500) {
        Serial.printf("Progress: %u%%\n", (progress / (total / 100)));
        last_ota_time = millis();
      }
    })
    .onError([](ota_error_t error) {
      Serial.printf("Error[%u]: ", error);
      if (error == OTA_AUTH_ERROR) {
        Serial.println("Auth Failed");
      } else if (error == OTA_BEGIN_ERROR) {
        Serial.println("Begin Failed");
      } else if (error == OTA_CONNECT_ERROR) {
        Serial.println("Connect Failed");
      } else if (error == OTA_RECEIVE_ERROR) {
        Serial.println("Receive Failed");
      } else if (error == OTA_END_ERROR) {
        Serial.println("End Failed");
      }
    });
  ArduinoOTA.begin();

  Serial.println("Setup done");
}

void loop() {
  ArduinoOTA.handle();

  //Serial.println("Loop");
  delay(500);
  digitalWrite(LED,HIGH);
  delay(500);
  digitalWrite(LED,LOW);
}