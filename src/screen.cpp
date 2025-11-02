#include "screen.hpp"

#include <algorithm>
#include <cstdio>
#include <cstdlib>

static uint8_t progress_char_base[16] = {
        0, 0, 0, 0,
        0, 0, 0, 0,
        255, 255, 255, 255,
        255, 255, 255, 255
    };

namespace {

// Convert the motor state id into a compact single-character glyph so the
// diagnostics view can show it alongside other numbers.
char state_symbol(int state) {
    switch (state) {
    case 0:
        return 'I'; // Idle
    case 1:
        return '^'; // RampUp
    case 2:
        return '>'; // Running
    case 3:
        return 'v'; // RampDown
    case 4:
        return '='; // Coast
    default:
        return '?';
    }
}

// Small helper so we can quickly mirror the direction on Screen B.
char direction_symbol(int direction) {
    return direction >= 0 ? '>' : '<';
}

// Clamp a value into a printable range without introducing std::clamp
// dependencies in the header.
int clamp_to_range(int value, int min, int max) {
    return std::min(std::max(value, min), max);
}

// Render the signed pulse counter with either raw counts or a scaled
// k-suffix once the value grows beyond 5 digits (fits 7 chars total).
void format_count(char* dest, size_t len, int32_t count) {
    const char sign = count >= 0 ? '+' : '-';
    uint32_t magnitude = static_cast<uint32_t>(std::abs(count));
    if (magnitude >= 10000) {
        uint32_t whole = magnitude / 1000;
        uint32_t tenths = (magnitude % 1000) / 100;
        std::snprintf(dest, len, "%c%2lu.%1luk",
            sign,
            static_cast<unsigned long>(whole),
            static_cast<unsigned long>(tenths));
    } else {
        std::snprintf(dest, len, "%c%4lu",
            sign,
            static_cast<unsigned long>(magnitude));
    }
}

// Turn milliseconds into a fixed-width seconds.tenths value (xx.xs) so
// the row stays aligned while still showing some fractional detail.
void format_duration(char* dest, size_t len, uint32_t ms) {
    uint32_t tenths = (ms + 50) / 100; // round to nearest 0.1 second
    uint32_t seconds = tenths / 10;
    uint32_t tenth = tenths % 10;
    std::snprintf(dest, len, "%2lu.%1lus",
        static_cast<unsigned long>(seconds),
        static_cast<unsigned long>(tenth));
}

} // namespace

Screen::Screen()
#ifdef ARDUINO
    : lcd(PCF8574_ADDR_A21_A11_A01, 4, 5, 6, 16, 11, 12, 13, 14, POSITIVE),
      screen(ID::A),
#else
    : screen(ID::A),
#endif
      customCharsLoaded(false),
      initialized(false),
      screenStackSize(0) {
    lineBuffer[0].reserve(16);
    lineBuffer[1].reserve(16);
}

bool Screen::begin() {
    customCharsLoaded = false;
#ifdef ARDUINO
    initialized = (lcd.begin(16, 2, LCD_5x8DOTS) == 1);
    if (!initialized) {
        return false;
    }
    lcd.clear();
#else
    initialized = true;
#endif
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
#ifdef ARDUINO
    lcd.clear();
#endif
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
#ifdef ARDUINO
    if (initialized) {
        lcd.clear();
    }
#endif
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
    // Diagnostics screen cycles through sub-pages to expose more metrics
    // than fit on a single 16x2 LCD row.
    const int page_index = page % 3;
    char row0[17];
    char row1[17];

    switch (page_index) {
    case 0: {
        // Page 0: live control loop snapshot (RPM vs target, duty, PID state).
        std::snprintf(row0, sizeof(row0), "R%3d/%3d D%3d S%c",
            clamp_to_range(rpm, -999, 999),
            clamp_to_range(targetRpm, -999, 999),
            clamp_to_range(duty, 0, 999),
            state_symbol(motorState));
        std::snprintf(row1, sizeof(row1), "I%+03d R%03d P%+03d",
            clamp_to_range(pidIntegral, -99, 99),
            clamp_to_range(targetRotation, 0, 999),
            clamp_to_range(targetProgress, -99, 999));
        writeText(0, 0, String(row0));
        writeText(1, 0, String(row1));
        break;
    }
    case 1: {
        // Page 1: encoder totals, direction and control-loop error history.
        char count_buffer[8];
        format_count(count_buffer, sizeof(count_buffer), totalCount);
        std::snprintf(row0, sizeof(row0), "CNT%s DIR%c",
            count_buffer,
            direction_symbol(totalDirection));
        std::snprintf(row1, sizeof(row1), "AGE%04u ERR%+03d",
            static_cast<unsigned>(std::min<uint32_t>(stateAgeMs, static_cast<uint32_t>(9999))),
            clamp_to_range(pidError, -99, 99));
        writeText(0, 0, String(row0));
        writeText(1, 0, String(row1));
        break;
    }
    case 2: {
        // Page 2: recent cycle timing plus forward/backward stroke advances.
        char last_buffer[8];
        char prev_buffer[8];
        format_duration(last_buffer, sizeof(last_buffer), lastCycleMs);
        format_duration(prev_buffer, sizeof(prev_buffer), prevCycleMs);
        std::snprintf(row0, sizeof(row0), "C%s P%s",
            last_buffer,
            prev_buffer);
        std::snprintf(row1, sizeof(row1), "FW%03d BK%+04d",
            clamp_to_range(lastForwardDegrees, 0, 999),
            clamp_to_range(-lastBackwardDegrees, -999, 0));
        writeText(0, 0, String(row0));
        writeText(1, 0, String(row1));
        break;
    }
    default:
        break;
    }
}

void Screen::renderScreenC() {
    // Reserved for future implementation
}

void Screen::renderScreenD() {
    const int page_index = page % 3;
    char row0[17];
    char row1[17];

    switch (page_index) {
    case 0: {
        // Page 0: WiFi SSID and IP to confirm network attachment.
        String ssid = wifiSsid.length() > 0 ? wifiSsid : String("n/a");
        if (ssid.length() > 11) {
            ssid = ssid.substring(0, 11);
        }
        String ip = wifiIp.length() > 0 ? wifiIp : String("0.0.0.0");
        if (ip.length() > 16) {
            ip = ip.substring(0, 16);
        }
        std::snprintf(row0, sizeof(row0), "WiFi:%-11s", ssid.c_str());
        std::snprintf(row1, sizeof(row1), "%-16s", ip.c_str());
        writeText(0, 0, String(row0));
        writeText(1, 0, String(row1));
        break;
    }
    case 1: {
        // Page 1: MQTT broker identity plus live connection state.
        String host = mqttHost.length() > 0 ? mqttHost : String("n/a");
        if (host.length() > 11) {
            host = host.substring(0, 11);
        }
        const char* status = mqttConnected ? "Connected" : "Disconnected";
        std::snprintf(row0, sizeof(row0), "MQTT:%-11s", status);
        std::snprintf(row1, sizeof(row1), "%-16s", host.c_str());
        writeText(0, 0, String(row0));
        writeText(1, 0, String(row1));
        break;
    }
    case 2: {
        // Page 2: Build metadata so operators know the flashed firmware.
        String date = buildDate.length() > 0 ? buildDate : String(__DATE__);
        if (date.length() > 16) {
            date = date.substring(0, 16);
        }
        String time = buildTime.length() > 0 ? buildTime : String(__TIME__);
        if (time.length() > 16) {
            time = time.substring(0, 16);
        }
        String row0Text = "Bld:" + date;
        String row1Text = "Time:" + time;
        if (row0Text.length() > 16) {
            row0Text = row0Text.substring(0, 16);
        }
        if (row1Text.length() > 16) {
            row1Text = row1Text.substring(0, 16);
        }
        while (row0Text.length() < 16) {
            row0Text += ' ';
        }
        while (row1Text.length() < 16) {
            row1Text += ' ';
        }
        writeText(0, 0, row0Text);
        writeText(1, 0, row1Text);
        break;
    }
    default:
        break;
    }
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
#ifdef ARDUINO
    lcd.setCursor(0, row);
    lcd.print(lineBuffer[row]);
#else
    (void)row;
#endif
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
    if (minutes > 99) {
        minutes = 99; // display is limited to two minute digits
    }
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
#ifdef ARDUINO
    for (int i = 0; i < 8; i++) {
        lcd.createChar(static_cast<uint8_t>(i), progress_char_base + i);
    }
#endif
    customCharsLoaded = true;
}
