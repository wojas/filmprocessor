#include <Arduino.h>
#include <WiFi.h>
#include <ArduinoOTA.h>
#include <PubSubClient.h>
#include "secrets.h"

#include <Wire.h>
#include <LiquidCrystal_I2C.h>

#include "motor.h"

#define LED 2

const char *ssid = SECRET_WIFI_SSID;
const char *password = SECRET_WIFI_PASS;
uint32_t last_ota_time = 0;

WiFiClient espClient;
PubSubClient client(espClient);
unsigned long lastMsg = 0;
#define MSG_BUFFER_SIZE	(80)
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


// FIXME: What are these params?
// https://registry.platformio.org/libraries/enjoyneering/LiquidCrystal_I2C/examples/HelloWorld/HelloWorld.ino
LiquidCrystal_I2C lcd(PCF8574_ADDR_A21_A11_A01, 4, 5, 6, 16, 11, 12, 13, 14, POSITIVE);
//LiquidCrystal_I2C lcd(PCF8574A_ADDR_A21_A11_A01, 4, 5, 6, 16, 11, 12, 13, 14, POSITIVE);

uint32_t start_millis = 0;
uint32_t last_reversal = 0;
uint32_t last_lcd = 0;

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
        type = "firmware";
      } else {  // U_SPIFFS
        type = "filesystem";
      }

      // Stop the motor
      motor_target_duty(0);

      // NOTE: if updating SPIFFS this would be the place to unmount SPIFFS using SPIFFS.end()
      Serial.println("Start updating " + type);
      lcd.clear();
      lcd.print("Update " + type);
    })
    .onEnd([]() {
      Serial.println("\nEnd");
      lcd.print("Update done");
    })
    .onProgress([](unsigned int progress, unsigned int total) {
      if (millis() - last_ota_time > 500) {
        Serial.printf("Progress: %u%%\n", (progress / (total / 100)));
        last_ota_time = millis();
      }
      lcd.setCursor(0, 1);
      // Need uint16
      uint16_t pct = 100 * progress / total;
      lcd.printHorizontalGraph('*', 1, pct, 100);
    })
    .onError([](ota_error_t error) {
      const char * err = "Unknown error";
      if (error == OTA_AUTH_ERROR) {
        err = "Auth Failed";
      } else if (error == OTA_BEGIN_ERROR) {
        err = "Begin Failed";
      } else if (error == OTA_CONNECT_ERROR) {
        err = "Connect Failed";
      } else if (error == OTA_RECEIVE_ERROR) {
        err = "Receive Failed";
      } else if (error == OTA_END_ERROR) {
        err = "End Failed";
      }
      Serial.printf("Error[%u]: %s", error, err);
      lcd.clear();
      lcd.printf("Error[%u]\n%s", error, err);
    });
  ArduinoOTA.begin();

  motor_init();

  Serial.println("Setting up LCD");
  //if (lcd.begin(16, 2, LCD_5x8DOTS, 400000, 250) != 1) //colums, rows, characters size
  if (lcd.begin(16, 2, LCD_5x8DOTS) != 1) //colums, rows, characters size
  {
    Serial.println(F("PCF8574 is not connected or lcd pins declaration is wrong. Only pins numbers: 4,5,6,16,11,12,13,14 are legal."));
    //delay(5000);
  } else {
    unsigned long t0 = millis();
    lcd.clear();
    lcd.print(F("Let's roll!"));
    unsigned long t1 = millis();
    Serial.print("LCD write took in ms: ");
    Serial.println(t1 - t0);
  }

  motor_target_duty(100);
  motor_target_rotation_per_cycle(720);

  start_millis = millis();
  last_reversal = start_millis;
  Serial.println("Setup done");
}

int last_dt = 0;

void loop() {
  //Serial.println("loop");

  ArduinoOTA.handle();

  // FIXME: Cannot block in our app!
  if (client.connected()) {
    client.loop();
  }

  uint32_t now = millis();
  // TODO: in background task
  if (now - lastMsg > 2000) {
    motor_dump_status();
    if (!client.connected()) {
      Serial.println("MQTT connect");
      mqtt_reconnect();
      Serial.println("MQTT connect finished");
    }
    if (client.connected()) {
      lastMsg = now;
      ++value;
      snprintf(msg, MSG_BUFFER_SIZE, "Motor: %d RPM, %d deg, %d duty",
        motor_rpm(), motor_position_degrees(), motor_duty());
      Serial.print("Publish message: ");
      Serial.println(msg);
      client.publish("outTopic", msg);
    }
  }

  digitalWrite(LED,motor_is_reversed() ? LOW : HIGH);

  //Serial.println("- duty on");
  //auto t0 = millis();
  //motor_duty(100);
  //delay(100);
  //auto t1 = millis();

  //last_dt = static_cast<int>(t1 - t0);

  //Serial.println("- duty 0");
  //motor_duty(0);
  //Serial.println("- delay 200ms ");
  //delay(100);
  //Serial.println("motor handling done");

  if (now - last_lcd > 200) {
    //lcd.clear();
    lcd.setCursor(0, 0);
    lcd.printf("%3d/%2d %3d %3d",
        motor_rpm(),
        motor_get_target_rpm(),
        motor_position_degrees(),
        motor_duty()
        );
    auto seconds = (now - start_millis) / 1000;
    auto minutes = seconds / 60;
    seconds = seconds % 60;
    lcd.setCursor(0, 1);
    lcd.printf("%2d:%02d", minutes, seconds);
    last_lcd = now;
  }

  //Serial.println("loop done");
  delay(100);
}