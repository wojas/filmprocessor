#pragma once

#include <Arduino.h>
#include <LiquidCrystal_I2C.h>

class Screen {
public:
    enum class Id {
        A,
        B,
        C,
        D,
    };

    Screen();

    bool begin();
    void setScreen(Id id);
    Id currentScreen() const { return screen; }
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

    // Screen B data
    String statusLine;
    String infoLine;

    // Screen C data
    String otaHeadline;
    uint16_t otaPercent = 0;

    // Screen D data
    String alertTitle;
    String alertDetail;

private:
    void renderScreenA();
    void renderScreenB();
    void renderScreenC();
    void renderScreenD();

    void clearRow(int row);
    void writeRow(int row, const String& text);
    void printTime(uint32_t seconds);
    char progressChar(int idx) const;
    void ensureCustomChars();

    LiquidCrystal_I2C lcd;
    Id screen;
    bool customCharsLoaded;
};
