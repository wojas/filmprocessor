#pragma once

#ifdef ARDUINO
#include <Arduino.h>
#include <LiquidCrystal_I2C.h>
#else
#include "Arduino.h"
#endif

#include <array>

class Screen {
public:
    enum class ID {
        A,
        B,
        C,
        D,
        Boot,
        Error,
        OTA,
    };

    Screen();

    bool begin();
    void setScreen(ID id);
    void pushScreen(ID id);
    bool popScreen();
    ID currentScreen() const { return screen; }
    void render();
    const String& bufferRow(int row) const { return lineBuffer[row]; }

    // Screen A data
    bool paused = true;
    int rpm = 0;
    int targetRpm = 0;
    int duty = 0;
    int progressDegrees = 0;
    uint32_t elapsedSeconds = 0;
    uint32_t previousCycleSeconds = 0;
    String keypadBuffer;
    int targetRotation = 0;        // target rotation per stroke (deg)
    int targetProgress = 0;        // delta degrees applied for net progress
    int pidIntegral = 0;           // integral accumulator for duty loop
    int pidError = 0;              // instantaneous RPM error
    int32_t totalCount = 0;        // signed encoder pulse count
    int totalDirection = 1;        // current direction (+/- 1)
    int motorState = 0;            // state machine id for Screen B glyphs
    uint32_t stateAgeMs = 0;       // time spent in current state
    uint32_t lastCycleMs = 0;      // duration of the most recent full cycle
    uint32_t prevCycleMs = 0;      // duration of the previous cycle
    int lastForwardDegrees = 0;    // forward stroke degrees in last cycle
    int lastBackwardDegrees = 0;   // reverse stroke degrees (absolute value)

    // Screen Boot data
    String bootStatus;
    String bootInfo;

    // Screen Error data
    String errorStatus;
    String errorInfo;

    // Screen OTA data
    String otaHeadline;
    uint16_t otaPercent = 0;

    // Pagination
    int page = 0;

private:
    void renderScreenA();
    void renderScreenB();
    void renderScreenC();
    void renderScreenD();
    void renderScreenBoot();
    void renderScreenError();
    void renderScreenOTA();

    void prepareBuffers();
    void flushLine(int row);
    void flushLines();
    void writeText(int row, int col, const String& text);
    void writeChar(int row, int col, char value);
    String formatTime(uint32_t seconds) const;
    char progressChar(int idx) const;
    void ensureCustomChars();

#ifdef ARDUINO
    LiquidCrystal_I2C lcd;
#endif
    ID screen;
    bool customCharsLoaded;
    bool initialized;
    String lineBuffer[2];
    static constexpr size_t screenStackCapacity = 8;
    struct ScreenState {
        ID id;
        int page;
    };
    std::array<ScreenState, screenStackCapacity> screenStack;
    size_t screenStackSize;
};
