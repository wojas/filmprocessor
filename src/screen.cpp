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
      screen(Id::A),
      customCharsLoaded(false) {
}

bool Screen::begin() {
    if (lcd.begin(16, 2, LCD_5x8DOTS) != 1) {
        return false;
    }
    lcd.clear();
    ensureCustomChars();
    return true;
}

void Screen::setScreen(Id id) {
    if (screen == id) {
        return;
    }
    screen = id;
    lcd.clear();
}

void Screen::render() {
    ensureCustomChars();
    switch (screen) {
    case Id::A:
        renderScreenA();
        break;
    case Id::B:
        renderScreenB();
        break;
    case Id::C:
        renderScreenC();
        break;
    case Id::D:
        renderScreenD();
        break;
    }
}

void Screen::renderScreenA() {
    clearRow(0);
    lcd.setCursor(0, 0);
    char dutyChar = duty / 32 + 1;
    if (duty == 0) {
        dutyChar = 0;
    }
    if (paused && rpm == 0 && duty == 0) {
        lcd.printf("[P]/%2d D%3d%c", targetRpm, duty, dutyChar);
    } else {
        lcd.printf("%3d/%2d D%3d%c", rpm, targetRpm, duty, dutyChar);
    }
    lcd.setCursor(15, 0);
    int progress = progressDegrees / 40;
    lcd.write(progressChar(progress));

    clearRow(1);
    lcd.setCursor(0, 1);
    printTime(elapsedSeconds);
    if (previousCycleSeconds > 0) {
        lcd.setCursor(6, 1);
        printTime(previousCycleSeconds);
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
        lcd.setCursor(col, 1);
        lcd.print(buf);
    }
}

void Screen::renderScreenB() {
    writeRow(0, statusLine);
    writeRow(1, infoLine);
}

void Screen::renderScreenC() {
    writeRow(0, otaHeadline);
    clearRow(1);
    lcd.printHorizontalGraph('*', 1, otaPercent, 100);
}

void Screen::renderScreenD() {
    writeRow(0, alertTitle);
    writeRow(1, alertDetail);
}

void Screen::clearRow(int row) {
    lcd.setCursor(0, row);
    lcd.print(F("                "));
    lcd.setCursor(0, row);
}

void Screen::writeRow(int row, const String& text) {
    clearRow(row);
    String trimmed = text;
    if (trimmed.length() > 16) {
        trimmed = trimmed.substring(0, 16);
    }
    lcd.print(trimmed);
}

void Screen::printTime(uint32_t seconds) {
    uint32_t minutes = seconds / 60;
    uint32_t remainder = seconds % 60;
    char buffer[6];
    snprintf(buffer, sizeof(buffer), "%2lu:%02lu",
        static_cast<unsigned long>(minutes),
        static_cast<unsigned long>(remainder));
    lcd.print(buffer);
}

char Screen::progressChar(int idx) const {
    return idx < 8 ? static_cast<char>(idx) : static_cast<char>(0xFF);
}

void Screen::ensureCustomChars() {
    if (customCharsLoaded) {
        return;
    }
    for (int i = 0; i < 8; i++) {
        lcd.createChar(i, progress_char_base + i);
    }
    customCharsLoaded = true;
}
