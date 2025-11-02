#include <unity.h>
#include <string>

#include "screen.hpp"

#ifndef ARDUINO
#include "../../src/screen.cpp"
#endif
static void test_boot_screen_renders_without_hardware() {
    Screen screen;
    TEST_ASSERT_TRUE(screen.begin());

    screen.bootStatus = "Hello";
    screen.bootInfo = "World";
    screen.setScreen(Screen::ID::Boot);
    screen.render();

    std::string row0 = screen.bufferRow(0).c_str();
    std::string row1 = screen.bufferRow(1).c_str();
    row0.resize(16, ' ');
    row1.resize(16, ' ');
    TEST_ASSERT_EQUAL_STRING_LEN("Hello           ", row0.c_str(), 16);
    TEST_ASSERT_EQUAL_STRING_LEN("World           ", row1.c_str(), 16);
}

static void test_screen_a_formats_basic_state() {
    Screen screen;
    TEST_ASSERT_TRUE(screen.begin());

    screen.paused = true;
    screen.rpm = 0;
    screen.targetRpm = 12;
    screen.duty = 0;
    screen.progressDegrees = 0;
    screen.elapsedSeconds = 125;
    screen.previousCycleSeconds = 90;
    screen.keypadBuffer = "42";
    screen.setScreen(Screen::ID::A);
    screen.render();

    std::string row0 = screen.bufferRow(0).c_str();
    std::string row1 = screen.bufferRow(1).c_str();
    row0.resize(16, ' ');
    row1.resize(16, ' ');
    TEST_ASSERT_EQUAL(16, static_cast<int>(row0.size()));
    TEST_ASSERT_EQUAL_CHAR_ARRAY("[P]/12 D  0", row0.c_str(), 11);
    TEST_ASSERT_EQUAL_STRING_LEN(" 2:05  1:30   42", row1.c_str(), 16);
}

static void test_screen_d_pages_cycle() {
    Screen screen;
    TEST_ASSERT_TRUE(screen.begin());

    screen.wifiSsid = "StudioAP";
    screen.wifiIp = "192.168.1.42";
    screen.mqttHost = "broker.lab";
    screen.mqttPort = 1883;
    screen.mqttConnected = true;
    screen.buildDate = "2024-03-18";
    screen.buildTime = "12:34:56";

    screen.setScreen(Screen::ID::D);
    screen.render();
    std::string row0 = screen.bufferRow(0).c_str();
    std::string row1 = screen.bufferRow(1).c_str();
    row0.resize(16, ' ');
    row1.resize(16, ' ');
    TEST_ASSERT_EQUAL_STRING_LEN("WiFi:StudioAP   ", row0.c_str(), 16);
    TEST_ASSERT_EQUAL_STRING_LEN("192.168.1.42    ", row1.c_str(), 16);

    screen.setScreen(Screen::ID::D); // advance to page 1
    screen.render();
    row0 = screen.bufferRow(0).c_str();
    row1 = screen.bufferRow(1).c_str();
    row0.resize(16, ' ');
    row1.resize(16, ' ');
    TEST_ASSERT_EQUAL_STRING_LEN("MQTT:broker.lab ", row0.c_str(), 16);
    TEST_ASSERT_EQUAL_STRING_LEN("Connected       ", row1.c_str(), 16);

    screen.setScreen(Screen::ID::D); // advance to page 2
    screen.render();
    row0 = screen.bufferRow(0).c_str();
    row1 = screen.bufferRow(1).c_str();
    row0.resize(16, ' ');
    row1.resize(16, ' ');
    TEST_ASSERT_EQUAL_STRING_LEN("Bld:2024-03-18  ", row0.c_str(), 16);
    TEST_ASSERT_EQUAL_STRING_LEN("Time:12:34:56   ", row1.c_str(), 16);
}

void setUp() {}

void tearDown() {}

static int run() {
    UNITY_BEGIN();
    RUN_TEST(test_boot_screen_renders_without_hardware);
    RUN_TEST(test_screen_a_formats_basic_state);
    RUN_TEST(test_screen_d_pages_cycle);
    return UNITY_END();
}

#ifdef ARDUINO
void setup() {
    (void)run();
}

void loop() {}
#else
int main() {
    return run();
}
#endif
