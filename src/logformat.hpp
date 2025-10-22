#pragma once

#include <Arduino.h>

static int logformat_append_prefix(char* buf, int len, bool local = false) {
    unsigned long ts = millis();
    int sec = ts / 1000;
    int msec = ts % 1000;
    char flag = local ? '*' : ' ';
    return snprintf(buf, len, "[%5d.%03d]%c", sec, msec, flag);
};