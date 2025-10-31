#include "screen.hpp"

#include <cstdio>

static uint8_t progress_char_base[16] = {
        0, 0, 0, 0,
        0, 0, 0, 0,
        255, 255, 255, 255,
        255, 255, 255, 255
    };

Screen::Screen()
    : lcd(PCF8574_ADDR_A21_A11_A01, 4, 5, 6, 16, 11, 12, 13, 14, POSITIVE),
      screen(ID::A),
      customCharsLoaded(false),
      initialized(false),
      screenStackSize(0) {
    lineBuffer[0].reserve(16);
    lineBuffer[1].reserve(16);
}

bool Screen::begin() {
    customCharsLoaded = false;
    initialized = (lcd.begin(16, 2, LCD_5x8DOTS) == 1);
    if (!initialized) {
        return false;
    }
    lcd.clear();
    ensureCustomChars();
    return true;
}

void Screen::setScreen(ID id) {
    bool sameScreen = (screen == id);
    if (sameScreen) {
        ++page;
    } else {
        screen = id;
        page = 0;
    }
    if (!initialized) {
        return;
    }
    lcd.clear();
}

void Screen::pushScreen(ID id) {
    if (screenStackSize == screenStackCapacity) {
        for (size_t i = 1; i < screenStackCapacity; ++i) {
            screenStack[i - 1] = screenStack[i];
        }
        screenStackSize = screenStackCapacity - 1;
    }
    screenStack[screenStackSize++] = ScreenState{screen, page};
    setScreen(id);
}

bool Screen::popScreen() {
    if (screenStackSize == 0) {
        return false;
    }
    ScreenState previous = screenStack[--screenStackSize];
    screen = previous.id;
    page = previous.page;
    if (initialized) {
        lcd.clear();
    }
    return true;
}

void Screen::render() {
    if (!initialized) {
        return;
    }
    ensureCustomChars();
    prepareBuffers();
    switch (screen) {
    case ID::A:
        renderScreenA();
        break;
    case ID::B:
        renderScreenB();
        break;
    case ID::C:
        renderScreenC();
        break;
    case ID::D:
        renderScreenD();
        break;
    case ID::Boot:
        renderScreenBoot();
        break;
    case ID::Error:
        renderScreenError();
        break;
    case ID::OTA:
        renderScreenOTA();
        break;
    }
    flushLines();
}

void Screen::renderScreenA() {
    char baseLine[16];
    if (paused && rpm == 0 && duty == 0) {
        snprintf(baseLine, sizeof(baseLine), "[P]/%2d D%3d ", targetRpm, duty);
    } else {
        snprintf(baseLine, sizeof(baseLine), "%3d/%2d D%3d ", rpm, targetRpm, duty);
    }
    writeText(0, 0, String(baseLine));
    int dutyGauge = duty == 0 ? 0 : (duty / 32 + 1);
    if (dutyGauge > 8) {
        dutyGauge = 8;
    }
    if (dutyGauge > 0) {
        writeChar(0, 14, progressChar(dutyGauge));
    }
    int progress = progressDegrees / 40;
    if (progress > 8) {
        progress = 8;
    }
    if (progress > 0) {
        writeChar(0, 15, progressChar(progress));
    }

    writeText(1, 0, formatTime(elapsedSeconds));
    if (previousCycleSeconds > 0) {
        writeText(1, 6, formatTime(previousCycleSeconds));
    }
    if (!keypadBuffer.isEmpty()) {
        String buf = keypadBuffer;
        if (buf.length() > 8) {
            buf = buf.substring(buf.length() - 8);
        }
        int col = 16 - buf.length();
        if (col < 8) {
            col = 8;
        }
        writeText(1, col, buf);
    }
}

void Screen::renderScreenB() {
    // Reserved for future implementation
}

void Screen::renderScreenC() {
    // Reserved for future implementation
}

void Screen::renderScreenD() {
    // Reserved for future implementation
}

void Screen::renderScreenBoot() {
    writeText(0, 0, bootStatus);
    writeText(1, 0, bootInfo);
}

void Screen::renderScreenError() {
    writeText(0, 0, errorStatus);
    writeText(1, 0, errorInfo);
}

void Screen::renderScreenOTA() {
    writeText(0, 0, otaHeadline);
    uint16_t capped = otaPercent > 100 ? 100 : otaPercent;
    uint32_t units = static_cast<uint32_t>(capped) * 16 * 8 / 100;
    for (int col = 0; col < 16; ++col) {
        int level = static_cast<int>(units) - col * 8;
        if (level <= 0) {
            continue;
        }
        if (level >= 8) {
            writeChar(1, col, progressChar(8));
        } else {
            writeChar(1, col, progressChar(level));
        }
    }
}

void Screen::prepareBuffers() {
    lineBuffer[0] = F("                ");
    lineBuffer[1] = F("                ");
}

void Screen::flushLine(int row) {
    lcd.setCursor(0, row);
    lcd.print(lineBuffer[row]);
}

void Screen::flushLines() {
    flushLine(0);
    flushLine(1);
}

void Screen::writeText(int row, int col, const String& text) {
    int len = text.length();
    for (int i = 0; i < len; ++i) {
        int idx = col + i;
        if (idx < 0 || idx >= 16) {
            break;
        }
        if (idx >= static_cast<int>(lineBuffer[row].length())) {
            break;
        }
        lineBuffer[row].setCharAt(idx, text[i]);
    }
}

void Screen::writeChar(int row, int col, char value) {
    if (col < 0 || col >= 16) {
        return;
    }
    if (col >= static_cast<int>(lineBuffer[row].length())) {
        return;
    }
    lineBuffer[row].setCharAt(col, value);
}

String Screen::formatTime(uint32_t seconds) const {
    uint32_t minutes = seconds / 60;
    uint32_t remainder = seconds % 60;
    char buffer[6];
    snprintf(buffer, sizeof(buffer), "%2lu:%02lu",
        static_cast<unsigned long>(minutes),
        static_cast<unsigned long>(remainder));
    return String(buffer);
}

char Screen::progressChar(int idx) const {
    return idx < 8 ? static_cast<char>(idx) : static_cast<char>(0xFF);
}

void Screen::ensureCustomChars() {
    if (!initialized || customCharsLoaded) {
        return;
    }
    for (int i = 0; i < 8; i++) {
        lcd.createChar(i, progress_char_base + i);
    }
    customCharsLoaded = true;
}
