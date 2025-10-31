#include <Arduino.h>
#include <WiFi.h>
#include <ArduinoOTA.h>
#include <PubSubClient.h>
#include "secrets.h"

#include <cstdio>
#include <Wire.h>
#include <Keypad.h>
#include <Pushbutton.h>

#include "screen.hpp"
#include "mqtt.hpp"
#include "logger.hpp"
#include "input_match.hpp"

#include "motor.h"

#define LED 2

#define BUTTON_ROLL 32
//#define BUTTON_TIME 35
#define BUTTON_TIME 33

Pushbutton buttonRoll(BUTTON_ROLL);
Pushbutton buttonTime(BUTTON_TIME);

const char* ssid = SECRET_WIFI_SSID;
const char* password = SECRET_WIFI_PASS;
uint32_t last_ota_time = 0;

Screen screen;

WiFiClient espClient;
//PubSubClient client(espClient);
unsigned long lastMsg = 0;
#define MSG_BUFFER_SIZE	(80)
char msg[MSG_BUFFER_SIZE];
int value = 0;

void mqtt_callback(const char* topic, const byte* payload, unsigned int length) {
    LOGF("[MQTT] Message topic=%s : %.*s", topic, length, payload);

    auto topicString = String(topic);

    if (topicString == "letsroll/reboot") {
        motor_target_duty(0);
        LOGF("[MQTT] reboot requested");
        screen.bootStatus = "MQTT reboot!";
        screen.bootInfo = "";
        screen.setScreen(Screen::ID::Boot);
        screen.render();
        delay(5000);
        ESP.restart();
        return;
    }

    // Below is all topic letsroll/control
    if (topicString != "letsroll/control") {
        LOGF("[MQTT] Unhandled topic: %s", topic);
        return;
    }
    if (length < 2) {
        return; // cannot be valid
    }
    auto s = String(payload, length);
    switch (byte command = payload[0]) {
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
    case 'G':
        {
            // Set progress per cycle
            int val = s.substring(1).toInt();
            motor_target_progress(val);
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


#define KP_ROW_NUM     4 // four rows
#define KP_COLUMN_NUM  4 // four columns
char keys[KP_ROW_NUM][KP_COLUMN_NUM] = {
        {'1', '2', '3', 'A'},
        {'4', '5', '6', 'B'},
        {'7', '8', '9', 'C'},
        {'*', '0', '#', 'D'}
    };
byte pin_rows[KP_ROW_NUM] = {15, 2, 0, 4};
byte pin_column[KP_COLUMN_NUM] = {16, 17, 5, 18};
Keypad keypad = Keypad(makeKeymap(keys), pin_rows, pin_column, KP_ROW_NUM, KP_COLUMN_NUM);

static InputMatch keypad_matcher;

void reset_timer();

static void init_keypad_routes() {
    keypad_matcher.match("Annn#", [](const InputMatch::Result& res) {
        if (res.has_number && res.number < 100) {
            motor_target_rpm(res.number);
        }
    });
    keypad_matcher.match("Bnnn#", [](const InputMatch::Result& res) {
        if (!res.has_number) {
            return;
        }
        int value = res.number;
        if (res.leading_zero && value > 0) {
            value = -value;
        }
        motor_target_progress(value);
    });
    keypad_matcher.match("Cnnn#", [](const InputMatch::Result& res) {
        if (res.has_number) {
            motor_target_rotation_per_cycle(res.number);
        }
    });
    keypad_matcher.match("Dnnn#", [](const InputMatch::Result& res) {
        if (res.has_number && res.number <= 255) {
            motor_target_duty(res.number);
        }
    });
    keypad_matcher.match("#", [](const InputMatch::Result&) {
        reset_timer();
    });
    keypad_matcher.match("0", [](const InputMatch::Result&) {
        motor_target_duty(0);
    });
    keypad_matcher.match("1", [](const InputMatch::Result&) {
        motor_target_rotation_per_cycle(900);
        motor_target_progress(50);
        motor_target_rpm(70);
    });
    keypad_matcher.match("2", [](const InputMatch::Result&) {
        motor_target_rotation_per_cycle(720);
        motor_target_progress(50);
        motor_target_rpm(50);
    });
    keypad_matcher.match("3", [](const InputMatch::Result&) {
        motor_target_rotation_per_cycle(360);
        motor_target_progress(50);
        motor_target_rpm(25);
    });
    keypad_matcher.match("4", [](const InputMatch::Result&) {
        motor_target_rotation_per_cycle(720);
        motor_target_progress(50);
        motor_target_rpm(60);
    });
    keypad_matcher.match("5", [](const InputMatch::Result&) {
        motor_target_rotation_per_cycle(360);
        motor_target_progress(50);
        motor_target_rpm(5);
    });
    keypad_matcher.match("6", [](const InputMatch::Result&) {
        motor_target_rotation_per_cycle(360);
        motor_target_progress(50);
        motor_target_rpm(10);
    });
    keypad_matcher.match("7", [](const InputMatch::Result&) {
        motor_target_rotation_per_cycle(900);
        motor_target_progress(50);
        motor_target_duty(230);
    });
}


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

void error_blink(const int count, const int on_delay, const int off_delay) {
    for (int i = 0; i < count; i++) {
        digitalWrite(LED, HIGH);
        delay(on_delay);
        digitalWrite(LED, LOW);
        delay(off_delay);
    }
}

WiFiClient mqttClient;

void setup() {
    Serial.begin(115200);
    pinMode(LED, OUTPUT);

    init_keypad_routes();

    // Setup LCD for early error output
    LOGF("Setting up LCD");
    unsigned long lcd_begin_start = millis();
    bool lcdReady = screen.begin();
    unsigned long lcd_begin_end = millis();
    if (!lcdReady) {
        LOGF("PCF8574 is not connected or lcd pins declaration is wrong. Only pins numbers: 4,5,6,16,11,12,13,14 are legal.");
        error_blink(10, 500, 200); // takes 7s
        //ESP.restart(); // FIXME: If not initialized, can it crash later when writing to it?
    } else {
        LOGF("LCD init took in ms: %lu", (lcd_begin_end - lcd_begin_start));
        screen.bootStatus = "Let's roll!";
        screen.bootInfo = "";
        screen.setScreen(Screen::ID::Boot);
        unsigned long lcd_write_start = millis();
        screen.render();
        unsigned long lcd_write_end = millis();
        LOGF("LCD write took in ms: %lu", (lcd_write_end - lcd_write_start));
    }

    // https://community.platformio.org/t/esp32-ota-using-platformio/15057/4
    screen.bootStatus = "WiFi:";
    screen.bootInfo = ssid;
    screen.setScreen(Screen::ID::Boot);
    screen.render();
    LOGF("[WiFi] Connecting to WiFI network %s", ssid);
    digitalWrite(LED,HIGH);
    const char* name = "filmprocessor";
    WiFi.setHostname(name);
    WiFi.mode(WIFI_STA);
    WiFi.begin(ssid, password);
    auto wifi_res = WiFi.waitForConnectResult();
    if (wifi_res != WL_CONNECTED) {
        digitalWrite(LED,LOW);
        Serial.println("[WiFi] Connection Failed!");
        char buffer[17];
        snprintf(buffer, sizeof(buffer), "WiFi err %3d", wifi_res);
        screen.bootStatus = buffer;
        screen.bootInfo = "";
        screen.render();

        error_blink(15, 200, 100); // takes 4.5s
        //ESP.restart();
    } else {
        // Display IP
        auto ip = WiFi.localIP();
        LOGF("[WiFi] connected, IP address: %s", ip.toString().c_str());

        // Display IP for 500ms
        screen.bootStatus = "WiFi ready";
        screen.bootInfo = ip.toString();
        screen.render();
        delay(500);
        screen.bootInfo = "";
        screen.render();
    }
    screen.setScreen(Screen::ID::A);
    screen.render();

    // Setup MQTT and logging (over serial, telnet, and mqtt)
    auto clientId = MQTT::genClientId(name);
    MQTT::begin(mqttClient,
        SECRET_MQTT_SERVER,
        1883,
        clientId.c_str(),
        SECRET_MQTT_USER,
        SECRET_MQTT_PASS);
    MQTT::startTask();

    Logger::enableMQTT("letsroll/log");
    Logger::beginServer();
    Logger::startTask();

    // Once connected, publish an announcement...
    MQTT::publishAsync("letsroll/hello", "hello world");
    MQTT::subscribe("letsroll/control");
    MQTT::subscribe("letsroll/reboot");

    // https://github.com/espressif/arduino-esp32/blob/master/libraries/ArduinoOTA/examples/BasicOTA/BasicOTA.ino
    ArduinoOTA.setPassword("thaal6aiJievee"); // FIXME: move to header
    ArduinoOTA
        .onStart([]()
        {
            String type;
            if (ArduinoOTA.getCommand() == U_FLASH) {
                type = "firmware";
            } else {
                // U_SPIFFS
                type = "filesystem";
            }

            // Stop the motor
            motor_target_duty(0);

            // NOTE: if updating SPIFFS this would be the place to unmount SPIFFS using SPIFFS.end()
            LOGF("[OTA] Start updating %s", type.c_str());
            screen.otaHeadline = "Update " + type;
            screen.otaPercent = 0;
            screen.pushScreen(Screen::ID::OTA);
            screen.render();
        })
        .onEnd([]()
        {
            Serial.println("\nEnd");
            screen.otaHeadline = "Update done";
            screen.otaPercent = 100;
            screen.render();
        })
        .onProgress([](unsigned int progress, unsigned int total)
        {
            if (millis() - last_ota_time > 500) {
                LOGF("[OTA] Progress: %u%%", (progress / (total / 100)));
                last_ota_time = millis();
            }
            // Need uint16
            uint16_t pct = 100 * progress / total;
            screen.otaPercent = pct;
            screen.render();
        })
        .onError([](ota_error_t error)
        {
            const char* err = "Unknown error";
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
            LOGF("[OTA] Error %u: %s", error, err);
            screen.errorStatus = String("OTA err ") + error;
            screen.errorInfo = err;
            screen.setScreen(Screen::ID::Error);
            screen.render();
            sleep(5);
            screen.popScreen();
        });
    ArduinoOTA.begin();

    // Default on startup after future unpause
    motor_target_rotation_per_cycle(720);
    motor_target_progress(50);
    motor_target_rpm(10);
    motor_init();

    start_millis = millis();
    last_reversal = start_millis;
    LOGF("setup() done");
}

int last_dt = 0;

void kp_handle(char key) {
    keypad_matcher.consume(key);
    screen.keypadBuffer = keypad_matcher.buffer().c_str();
    if (screen.currentScreen() == Screen::ID::A) {
        screen.render();
    }
}

void loop() {
    //Serial.println("loop");

    ArduinoOTA.handle();

    uint32_t now = millis();
    if (now - lastMsg > 10000) {
        motor_dump_status();
        lastMsg = now;
    }

    digitalWrite(LED, motor_is_reversed() ? LOW : HIGH);

    // MQTT incoming message handling
    MQTT::RxView msg;
    if (MQTT::recv(msg, 0)) {
        // handle one message
        LOGF("[MQTT] received %s : %.*s", msg.topic, msg.len, reinterpret_cast<const char*>(msg.payload));
        mqtt_callback(msg.topic, msg.payload, msg.len);
        MQTT::free(msg);  // IMPORTANT: return the slot to the pool
    }

    // LCD update
    if (now - last_lcd > 200) {
        screen.rpm = motor_rpm();
        screen.targetRpm = motor_get_target_rpm();
        screen.duty = motor_duty();
        screen.paused = motor_is_paused();
        screen.progressDegrees = static_cast<int>(motor_position_degrees());
        screen.elapsedSeconds = (now - start_millis) / 1000;
        screen.previousCycleSeconds = prev_time_sec;
        screen.render();
        last_lcd = now;
    }

    // Handle keypad input
    char key = keypad.getKey();
    if (key) {
        Serial.printf("Keypad: %c\n", key);
        kp_handle(key);
    }

    // Handle the ROLL and TIME button
    if (buttonRoll.getSingleDebouncedPress()) {
        bool is_paused = motor_toggle_paused();
        LOGF("Button: ROLL pressed, paused is now %d", is_paused);
    }
    if (buttonTime.getSingleDebouncedPress()) {
        reset_timer();
        LOGF("Button: TIME pressed");
    }

    //Serial.println("loop done");
    delay(10);
}
