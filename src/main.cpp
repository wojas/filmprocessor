#include <Arduino.h>
#include <WiFi.h>
#include <ArduinoOTA.h>
#include <PubSubClient.h>
#include "secrets.h"

#include <Wire.h>
#include <LiquidCrystal_I2C.h>
#include <Keypad.h>
#include <Pushbutton.h>

#include "motor.h"

#define LED 2

// https://registry.platformio.org/libraries/enjoyneering/LiquidCrystal_I2C/examples/HelloWorld/HelloWorld.ino
LiquidCrystal_I2C lcd(PCF8574_ADDR_A21_A11_A01, 4, 5, 6, 16, 11, 12, 13, 14, POSITIVE);
//LiquidCrystal_I2C lcd(PCF8574A_ADDR_A21_A11_A01, 4, 5, 6, 16, 11, 12, 13, 14, POSITIVE);

// LCD custom characters for progress made from this by pointing at the right offset
byte progress_char_base[16] = {
  0, 0, 0, 0,
  0, 0, 0, 0,
  255, 255, 255, 255,
  255, 255, 255, 255
};

#define BUTTON_ROLL 32
#define BUTTON_TIME 35

Pushbutton buttonRoll(BUTTON_ROLL);
Pushbutton buttonTime(BUTTON_TIME);

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

  auto topicString = String(topic);

  if (topic == "letsroll/reboot") {
    motor_target_duty(0);
    Serial.println("[MQTT] reboot requested");
    lcd.clear();
    lcd.setCursor(0, 0);
    lcd.print("MQTT reboot!");
    delay(5000);
    ESP.restart();
    return;
  }

  // Below is all topic letsroll/motor/control
  if (topicString != "letsroll/motor/control") {
    Serial.println("Unhandled topic");
    return;
  }
  if (length < 2) {
    return; // cannot be valid
  }
  auto s = String(reinterpret_cast<char*>(payload), length);
  byte command = payload[0];
  switch (command) {
  case 'R':
    {
      // Set RPM
      int val = s.substring(1).toInt();
      motor_target_rpm(val);
      break;
    }
  case 'D':
    {
      // Set duty
      int val = s.substring(1).toInt();
      motor_target_duty(val);
      break;
    }
  case 'C':
    {
      // Set rotation per cycle
      int val = s.substring(1).toInt();
      motor_target_rotation_per_cycle(val);
      break;
    }
  case 'P':
    {
      // Pause the motor
      bool pause = s.length() == 1 || s.substring(1).toInt() > 0;
      motor_set_paused(pause);
      break;
    }
  default:
    {
      Serial.println("[MQTT] Invalid command");
    }
  }
}

void mqtt_reconnect() {
    Serial.print("Attempting MQTT connection...");
    // Create a random client ID
    String clientId = "letsroll-";
    clientId += String(random(0xffff), HEX);
    // Attempt to connect
    if (client.connect(clientId.c_str(), SECRET_MQTT_USER, SECRET_MQTT_PASS)) {
      Serial.println("connected");
      // Once connected, publish an announcement...
      client.publish("letsroll/hello", "hello world");
      // ... and resubscribe
      client.subscribe("letsroll/motor/control");
      client.subscribe("letsroll/reboot");
    } else {
      Serial.print("failed, rc=");
      Serial.print(client.state());
      Serial.println(" try again later");
    }
}

#define KP_ROW_NUM     4 // four rows
#define KP_COLUMN_NUM  4 // four columns
char keys[KP_ROW_NUM][KP_COLUMN_NUM] = {
  {'1', '2', '3', 'A'},
  {'4', '5', '6', 'B'},
  {'7', '8', '9', 'C'},
  {'*', '0', '#', 'D'}
};
byte pin_rows[KP_ROW_NUM]      = {15, 2, 0, 4};
byte pin_column[KP_COLUMN_NUM] = {16, 17, 5, 18};
Keypad keypad = Keypad(makeKeymap(keys), pin_rows, pin_column, KP_ROW_NUM, KP_COLUMN_NUM );


uint32_t start_millis = 0;
uint32_t prev_time_sec = 0;
uint32_t last_reversal = 0;
uint32_t last_lcd = 0;

void reset_timer() {
  auto now = millis();
  auto prev = (now - start_millis) / 1000;
  // Ignore short times, likely mistakes or uninteresting for development
  if (prev > 15) {
    prev_time_sec = prev;
  }
  start_millis = now;
}

void lcd_print_time(uint32_t seconds) {
  auto minutes = seconds / 60;
  seconds = seconds % 60;
  lcd.printf("%2d:%02d", minutes, seconds);
}

void error_blink(const int count, const int on_delay, const int off_delay) {
  for (int i = 0; i < count; i++) {
    digitalWrite(LED, HIGH);
    delay(on_delay);
    digitalWrite(LED, LOW);
    delay(off_delay);
  }
}

void lcd_clear_row(const int row) {
  lcd.setCursor(0, row);
  lcd.print("                "); // 16 spaces
  lcd.setCursor(0, row);
}

void setup() {
  Serial.begin(115200);
  pinMode(LED,OUTPUT);

  // Setup LCD for early error output
  Serial.println("Setting up LCD");
  if (lcd.begin(16, 2, LCD_5x8DOTS) != 1) //colums, rows, characters size
  {
    Serial.println(F("PCF8574 is not connected or lcd pins declaration is wrong. Only pins numbers: 4,5,6,16,11,12,13,14 are legal."));
    error_blink(10, 500, 200); // takes 7s
    ESP.restart();
  }
  unsigned long t0 = millis();
  lcd.clear();
  lcd.print(F("Let's roll!"));
  unsigned long t1 = millis();
  Serial.print("LCD write took in ms: ");
  Serial.println(t1 - t0);
  for (int i = 0; i < 8; i++) {
    lcd.createChar(i, progress_char_base+i);
  }

  // https://community.platformio.org/t/esp32-ota-using-platformio/15057/4
  lcd_clear_row(1);
  lcd.print(F("WiFi:"));
  lcd.print(ssid);
  Serial.print("[WiFi] Connecting to WiFI network ");
  Serial.println(ssid);
  digitalWrite(LED,HIGH);
  WiFi.setHostname("filmprocessor");
  WiFi.mode(WIFI_STA);
  WiFi.begin(ssid, password);
  auto wifi_res = WiFi.waitForConnectResult();
  if (wifi_res != WL_CONNECTED) {
    digitalWrite(LED,LOW);
    Serial.println("[WiFi] Connection Failed!");
    lcd_clear_row(0);
    lcd.printf("WiFi err %3d    ", wifi_res);

    error_blink(15, 200, 100); // takes 4.5s
    //ESP.restart();
  } else {
    // Display IP
    auto ip = WiFi.localIP();
    Serial.println("[WiFI] connected");
    Serial.print("[WiFi] IP address: ");
    Serial.println(ip);

    // Display IP for 500ms
    lcd_clear_row(1);
    lcd.print(ip.toString());
    delay(500);
    lcd_clear_row(1);
  }

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
      lcd.setCursor(0, 0);
      lcd.printf("Error %u", error);
      lcd.setCursor(0, 1);
      lcd.print(err);
      sleep(5);
    });
  ArduinoOTA.begin();

  // Default on startup after future unpause
  motor_target_rotation_per_cycle(720);
  motor_target_rpm(10);
  motor_init();

  start_millis = millis();
  last_reversal = start_millis;
  Serial.println("Setup done");
}

int last_dt = 0;

void kp_handle(char key) {
  // Keypad state
  static char mode = 0;
  static uint16_t val = 0;
  static String buffer;
  char digit = 0;

  buffer += key;

  switch (mode) {
  case '*':
    // Special function on next number
    if (key == '*') {
      mode = 0;
      buffer.clear();
      break;
    }
    mode = 0; // reset mode after one char
    break;

  case 'A':
  case 'B':
  case 'C':
  case 'D':
    // In param mode (end with '#' or cancel with '*')
    if (key == '*') {
      mode = 0;
      val = 0;
      buffer.clear();
      break;
    }
    if (key == '#') {
      switch (mode) {
      case 'A':
        if (val < 100) {
          motor_target_rpm(val);
        }
        break;
      case 'B':
        break;
      case 'C':
        motor_target_rotation_per_cycle(val);
        break;
      case 'D':
        if (val <= 255) {
          motor_target_duty(val);
        }
        break;
      default:
        break;
      }
      mode = 0;
      val = 0;
      break;
    }
    // Input number
    digit = key - '0';
    if (digit >= 0 && digit <= 9) {
      val = val * 10 + digit;
    }
    break;

  default:
    {
      // Normal mode (select program or mode)
      buffer.clear();
      buffer += key;
      switch (key) {
      case '0':
        // This is different from pausing with the ROLL button.
        motor_target_duty(0);
        break;
      case '1':
        motor_target_rpm(50);
        break;
      case '2':
        motor_target_rpm(75);
        break;
      case '3':
        motor_target_rpm(25);
        break;
      case '5':
        motor_target_rpm(5);
        break;
      case '6':
        motor_target_rpm(10);
        break;
      case '7':
        motor_target_duty(255);
        break;
      case '#':
        // Reset timer
        reset_timer();
        break;
      case 'A':
      case 'B':
      case 'C':
      case 'D':
      case '*':
        mode = key;
        val = 0;
        break;
      default:
        mode = 0;
        val = 0;
        break;
      }
    }
  }
  lcd.setCursor(8, 1);
  lcd.print("        ");
  lcd.setCursor(16 - buffer.length(), 1);
  lcd.print(buffer);
}

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
      client.publish("letsroll/log", msg);
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
    int rpm = motor_rpm();
    int duty = motor_duty();
    if (motor_is_paused() && rpm == 0 && duty == 0) {
      lcd.printf("[P]/%2d D%3d", motor_get_target_rpm(), duty);
    } else {
      lcd.printf("%3d/%2d D%3d", rpm, motor_get_target_rpm(), duty);
    }
    // We defined custom progress characters 0-7 (vertical bar chart)
    // TODO: This should really be 0-8 (inclusive), with 0xFF being a full block.
    char progress = motor_position_degrees() / 45;
    lcd.setCursor(15, 0);
    lcd.write(progress);

    auto seconds = (now - start_millis) / 1000;
    lcd.setCursor(0, 1);
    lcd_print_time(seconds);
    if (prev_time_sec > 0) {
      lcd.setCursor(6, 1);
      lcd_print_time(prev_time_sec);
    }
    last_lcd = now;
  }

  // Handle keypad input
  char key = keypad.getKey();
  if (key) {
    Serial.printf("Keypad: %c\n", key);
    //lcd.setCursor(15, 1);
    //lcd.write(key);
    kp_handle(key);
  }

  // Handle the ROLL and TIME button
  if (buttonRoll.getSingleDebouncedPress()) {
    bool is_paused = motor_toggle_paused();
    Serial.printf("Button: ROLL pressed, paused is now %d\n", is_paused);
  }
  if (buttonTime.getSingleDebouncedPress()) {
    reset_timer();
    Serial.printf("Button: TIME pressed\n");
  }

  //Serial.println("loop done");
  delay(10);
}