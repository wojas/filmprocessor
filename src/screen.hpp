#pragma once

#include <Arduino.h>
#include <LiquidCrystal_I2C.h>
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
    /**
     * Pushes the current screen onto the stack and switches to the requested one.
     */
    void pushScreen(ID id);
    /**
     * Restores the previously active screen, if any.
     *
     * @return true when a previous screen was restored, false if the stack was empty.
     */
    bool popScreen();
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

    LiquidCrystal_I2C lcd;
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
