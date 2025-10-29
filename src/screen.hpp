#pragma once

#include <Arduino.h>
#include <LiquidCrystal_I2C.h>

class Screen {
public:
    enum class ID {
        A,
        B,
        C,
        D,
        Boot,
        OTA,
    };

    Screen();

    bool begin();
    void setScreen(ID id);
    ID currentScreen() const { return screen; }
    void render();

    // Screen A data
    bool paused = true;
    int rpm = 0;
    int targetRpm = 0;
    int duty = 0;
    int progressDegrees = 0;
    uint32_t elapsedSeconds = 0;
    uint32_t previousCycleSeconds = 0;
    String keypadBuffer;

    // Screen Boot data
    String bootStatus;
    String bootInfo;

    // Screen OTA data
    String otaHeadline;
    uint16_t otaPercent = 0;

private:
    void renderScreenA();
    void renderScreenB();
    void renderScreenC();
    void renderScreenD();
    void renderScreenBoot();
    void renderScreenOTA();

    void prepareBuffers();
    void flushLine(int row);
    void flushLines();
    void writeText(int row, int col, const String& text);
    void writeChar(int row, int col, char value);
    String formatTime(uint32_t seconds) const;
    char progressChar(int idx) const;
    void ensureCustomChars();

    LiquidCrystal_I2C lcd;
    ID screen;
    bool customCharsLoaded;
    bool initialized;
    String lineBuffer[2];
};
