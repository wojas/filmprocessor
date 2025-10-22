#pragma once

#include <Arduino.h>
#include <WiFi.h>

#include "logformat.hpp"
#include "freertos/FreeRTOS.h"
#include "freertos/queue.h"

#include "mqtt.hpp"

/*
  Logger — task-safe logging to both Serial and TCP clients.

  - One logger task owns all I/O (Serial + sockets).
  - Other tasks enqueue complete log records (lines or chunks) into a FreeRTOS queue.
  - Network input is ignored (monitor-only).
  - Slow/broken clients are dropped so logging never blocks the app.

  It expects Serial and WiFi (optional) to be setup by the caller.
  If WiFi is not available yet, it will just log to serial.
  If MQTT is configured, it will also log to MQTT.
*/
class Logger {
public:
    // These defaults use 16 kB for the buffering, out of the ESP32's 520 kB RAM,
    // plus an additional 4 kB for early logs before we have network logging.
    static constexpr size_t kMaxMsgBytes = 256; // max size of a single log message
    static constexpr uint8_t kMaxClients = 4; // simultaneous TCP viewers
    static constexpr uint16_t kDefaultPort = 23; // default TCP port (telnet)
    static constexpr uint16_t kQueueDepth = 64; // number of messages buffered
    static constexpr size_t kEarlyLogBytes = 4096; // early log replay buffer size
    static constexpr TickType_t kLogTick = pdMS_TO_TICKS(50); // interval we send one message

    struct Msg {
        uint16_t len; // payload length
        char buf[kMaxMsgBytes]; // payload bytes (not null-terminated)
    };

    // Call before beginServer.
    static void enableMQTT(const char* topic) {
        _earlyLogf("[LOG] MQTT topic: %s", topic);
        mqttTopic = topic;
    }

    static void beginServer(uint16_t port = kDefaultPort) {
        server = new WiFiServer(port);
        server->begin();
        server->setNoDelay(true);
        _earlyLogf("[LOG] TCP log server on port %d", port);
    }

    static void startTask(uint32_t stack = 4096, UBaseType_t prio = 3, BaseType_t core = APP_CPU_NUM) {
        if (!queue) {
            queue = xQueueCreate(kQueueDepth, sizeof(Msg));
        }
        xTaskCreatePinnedToCore(_taskEntry, "logger", stack, nullptr, prio, nullptr, core);
    }

    // printf-style (preferred; ensures line-atomic chunks you pass in)
    static void printf(const char* fmt, ...) {
        Msg m;
        va_list ap;
        va_start(ap, fmt);
        int prefix = logformat_append_prefix(m.buf, sizeof(m.buf));
        int n = vsnprintf(m.buf + prefix, sizeof(m.buf) - prefix, fmt, ap);
        int total = prefix + n;
        va_end(ap);
        if (n < 0) return;
        m.len = min(static_cast<int>(sizeof(m.buf)), total);
        if (!queue) {
            // Not yet initialized, early log.
            _earlyLogSize(m.buf, m.len);
            return;
        }
        const bool ok = xQueueSend(queue, &m, 0); // drop if full
        if (!ok) {
            droppedMessages++;
        }
    }

    // println convenience
    static void println(const String& s) {
        printf("%s\n", s.c_str());
    }
    static void println(const char* s) {
        printf("%s\n", s);
    }
    static void print(const String& s) {
        printf("%s", s.c_str());
    }
    static void print(const char* s) {
        printf("%s", s);
    }

    // ISR-safe minimal variant: enqueues a C string (truncated). Use sparingly.
    static void isrPrint(const char* s) {
        if (!queue) return;
        Msg m;
        size_t n = strnlen(s, sizeof(m.buf));
        memcpy(m.buf, s, n);
        m.len = n;
        BaseType_t hpw = pdFALSE;
        xQueueSendFromISR(queue, &m, &hpw);
        portYIELD_FROM_ISR(hpw);
    }

private:
    static void _taskEntry(void*) {
        _earlyLogf("[LOG] Logging task started");

        // Flush early logs to mqtt
        if (mqttTopic != nullptr) {
            int sent = 0;
            while (sent < earlyOffset) {
                int todo = earlyOffset - sent;
                int n = min(todo, MQTT_PAYLOAD_MAX);
                bool ok = MQTT::publishAsync(mqttTopic, reinterpret_cast<const uint8_t*>(earlyBuf + sent), n);
                sent += n;
            }
        }

        // Main loop
        for (;;) {
            _acceptClients();
            _pruneAndDrain();
            _flushOneMessage();
            vTaskDelay(kLogTick);
        }
    }

    static void _acceptClients() {
        if (!server) return;
        if (server->hasClient()) {
            for (auto& c : clients) {
                if (!c || !c.connected()) {
                    c = server->available();
                    c.setNoDelay(true);
                    // ignore any inbound data (monitor-only)
                    while (c.available()) c.read();
                    _earlyLogf("[LOG] Client connected (%s)", c.remoteIP().toString().c_str());
                    // Send early logs to client, ignore if it stalls here
                    c.write("--- early logs ---\n");
                    _writeAll(c, earlyBuf, earlyOffset);
                    c.write("--- early logs end ---\n");
                    return;
                }
            }
            // refuse extras
            WiFiClient junk = server->available();
            junk.stop();
        }
    }

    static void _pruneAndDrain() {
        for (auto& c : clients) {
            if (c && !c.connected()) {
                _earlyLogf("[LOG] Client disconnected");
                c.stop();
            } else {
                while (c && c.connected() && c.available()) c.read(); // ignore input
            }
        }
    }

    // Write a single message to serial and network
    static void _flushOneMessage() {
        if (!queue) return;
        Msg m;
        if (xQueueReceive(queue, &m, 0) != pdTRUE) return;

        // Append to early log as long as it is not full
        _earlyAppendSize(m.buf, m.len);

        // Always write to Serial
        Serial.write(m.buf, m.len);
        Serial.write("\n", 1);

        if (!WiFi.isConnected()) {
            return;
        }

        if (mqttTopic != nullptr) {
            MQTT::publishAsync(mqttTopic, reinterpret_cast<const uint8_t*>(m.buf), m.len);
        }

        // Then to all clients (non-blocking; drop slow/broken)
        for (auto& c : clients) {
            if (!c || !c.connected()) continue;
            if (!_writeAll(c, m.buf, m.len)) {
                // drop if stalled
                c.stop();
                return;
            }
            int wn = c.write("\n", 1);
            if (wn <= 0) {
                c.stop();
                return;
            }
        }
    }

    static bool _writeAll(WiFiClient& c, const char* buf, size_t len) {
        size_t off = 0;
        while (off < len) {
            int w = c.write(buf + off, len - off);
            if (w <= 0) {
                return false; // drop if stalled
            }
            off += w;
        }
        return true;
    }

    // Early logging to serial and buffer for later replay
    static void _earlyLogf(const char* fmt, ...) {
        char tmp[128];
        int prefix = logformat_append_prefix(tmp, sizeof(tmp));
        va_list ap;
        va_start(ap, fmt);
        int n = vsnprintf(tmp + prefix, sizeof(tmp) - prefix, fmt, ap);
        va_end(ap);
        int total = prefix + n;
        if (n > 0) {
            Serial.write(tmp, total);
            Serial.write("\n", 1);
            _earlyAppendSize(tmp, total);
        }
    }

    static void _earlyLogSize(const char* msg, size_t n) {
        if (msg[0] == 0) {
            return;
        }
        Serial.write(msg, n);
        Serial.write("\n", 1);
        _earlyAppendSize(msg, n);
    }

    static void _earlyAppendSize(const char* msg, size_t n) {
        // Write to early buffer for replay if still open
        if (earlyClosed) return;
        if (earlyOffset + n >= kEarlyLogBytes - 1) {
            earlyClosed = true;
            // For formatted output
            _earlyLogf("[LOG] Early logging buffer full");
            return;
        }
        memcpy(earlyBuf + earlyOffset, msg, n);
        earlyOffset += n;
        earlyBuf[earlyOffset] = '\n';
        earlyOffset++;
    }


private:
    static inline WiFiServer* server = nullptr;
    static inline WiFiClient clients[kMaxClients];
    static inline QueueHandle_t queue = nullptr;
    volatile static inline uint32_t droppedMessages = 0; // volatile is good enough for this
    static inline uint32_t lastDroppedMessages = 0;
    static inline const char* mqttTopic = nullptr;

    static inline bool earlyClosed = false;
    static inline char earlyBuf[kEarlyLogBytes];
    static inline int earlyOffset = 0;
};

// Convenience macros
#define LOGF(...)  Logger::printf(__VA_ARGS__)
//#define LOGLN(s)   Logger::println(s)
//#define LOG(s)     Logger::print(s)
