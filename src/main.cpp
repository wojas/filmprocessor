#include <Arduino.h>
#include <WiFi.h>
#include <ArduinoOTA.h>
#include <PubSubClient.h>
#include "secrets.h"

#include <Wire.h>
#include <LiquidCrystal_I2C.h>

#define LED 2

const char *ssid = SECRET_WIFI_SSID;
const char *password = SECRET_WIFI_PASS;
uint32_t last_ota_time = 0;

WiFiClient espClient;
PubSubClient client(espClient);
unsigned long lastMsg = 0;
#define MSG_BUFFER_SIZE	(50)
char msg[MSG_BUFFER_SIZE];
int value = 0;

void mqtt_callback(char* topic, byte* payload, unsigned int length) {
  Serial.print("[MQTT] Message arrived [");
  Serial.print(topic);
  Serial.print("] ");
  for (int i = 0; i < length; i++) {
    Serial.print((char)payload[i]);
  }
  Serial.println();
}

void mqtt_reconnect() {
    Serial.print("Attempting MQTT connection...");
    // Create a random client ID
    String clientId = "PubSubClient-";
    clientId += String(random(0xffff), HEX);
    // Attempt to connect
    if (client.connect(clientId.c_str(), SECRET_MQTT_USER, SECRET_MQTT_PASS)) {
      Serial.println("connected");
      // Once connected, publish an announcement...
      client.publish("outTopic", "hello world");
      // ... and resubscribe
      client.subscribe("inTopic");
    } else {
      Serial.print("failed, rc=");
      Serial.print(client.state());
      Serial.println(" try again later");
    }
}

// Define the GPIO pin for PWM (e.g., GPIO 14)
const int pwmPin = 14;
const int dirPin = 12;
// Define the PWM channel (0-15)
int pwmChannel = 0;
// Set PWM frequency (e.g., 25000 Hz)
// MAX14870 recommended max is 50kHz
int pwmFreq = 50000;
// Set PWM resolution (e.g., 8 bits for a 0-255 range)
int pwmRes = 8;

// FIXME: What are these params?
// https://registry.platformio.org/libraries/enjoyneering/LiquidCrystal_I2C/examples/HelloWorld/HelloWorld.ino
LiquidCrystal_I2C lcd(PCF8574_ADDR_A21_A11_A01, 4, 5, 6, 16, 11, 12, 13, 14, POSITIVE);
//LiquidCrystal_I2C lcd(PCF8574A_ADDR_A21_A11_A01, 4, 5, 6, 16, 11, 12, 13, 14, POSITIVE);

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

  client.setServer(SECRET_MQTT_SERVER, 1883);
  // For incoming messages, including headers. Default: 128
  client.setBufferSize(255);
  client.setCallback(mqtt_callback);

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

  // Set up the PWM channel
  ledcSetup(pwmChannel, pwmFreq, pwmRes);
  // Attach the PWM channel to the GPIO pin
  ledcAttachPin(pwmPin, pwmChannel);
  // Set the initial duty cycle to 0 (motor off)
  ledcWrite(pwmChannel, 0);

  pinMode(dirPin,OUTPUT);

  Serial.println("Setting up LCD");
  //if (lcd.begin(16, 2, LCD_5x8DOTS, 400000, 250) != 1) //colums, rows, characters size
  if (lcd.begin(16, 2, LCD_5x8DOTS) != 1) //colums, rows, characters size
  {
    Serial.println(F("PCF8574 is not connected or lcd pins declaration is wrong. Only pins numbers: 4,5,6,16,11,12,13,14 are legal."));
    //delay(5000);
  } else {
    unsigned long t0 = millis();
    lcd.print(F("Hello world!"));
    unsigned long t1 = millis();
    Serial.print("LCD write took in ms: ");
    Serial.println(t1 - t0);
  }

  Serial.println("Setup done");
}

bool reverse = false;

void loop() {
  ArduinoOTA.handle();

  // FIXME: Cannot block in our app!
  if (client.connected()) {
    client.loop();
  }

  unsigned long now = millis();
  if (now - lastMsg > 2000) {
    if (!client.connected()) {
      mqtt_reconnect();
    }
    if (client.connected()) {
      lastMsg = now;
      ++value;
      snprintf(msg, MSG_BUFFER_SIZE, "hello world #%d", value);
      lcd.clear();
      lcd.printf("Hello world #%d", value);
      Serial.print("Publish message: ");
      Serial.println(msg);
      client.publish("outTopic", msg);
    }
  }

  reverse = !reverse;
  digitalWrite(dirPin,reverse ? LOW : HIGH);
  digitalWrite(LED,reverse ? LOW : HIGH);


  //digitalWrite(LED,HIGH);

  // Example: Set motor speed to 50%
//  ledcWrite(pwmChannel, 255); // 128 is 50% of 255
  ledcWrite(pwmChannel, 100); // 128 is 50% of 255
  delay(300);
  delay(1700);

  // Example: Set motor speed to 0%
  //digitalWrite(LED,LOW);
  ledcWrite(pwmChannel, 0); // 0 is 0% of 255
  delay(200);


}