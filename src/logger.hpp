#pragma once

#include <Arduino.h>
#include <WiFi.h>
#include "freertos/FreeRTOS.h"
#include "freertos/queue.h"
#include "MQTT.hpp"

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
    // These defaults use 16 kB for the buffering, out of the ESP32's 520 kB RAM.
    static constexpr size_t kMaxMsgBytes = 256; // max size of a single log message
    static constexpr uint8_t kMaxClients = 4; // simultaneous TCP viewers
    static constexpr uint16_t kDefaultPort = 23; // default TCP port (telnet)
    static constexpr uint16_t kQueueDepth = 64; // number of messages buffered
    static constexpr TickType_t kLogTick = pdMS_TO_TICKS(50); // interval we send one message

    struct Msg {
        uint16_t len; // payload length
        char buf[kMaxMsgBytes]; // payload bytes (not null-terminated)
    };

    // Call before beginServer.
    static void enableMQTT(const char* topic) {
        Serial.printf("[LOG] MQTT topic: %s\n", topic);
        mqttTopic = topic;
    }

    static void beginServer(uint16_t port = kDefaultPort) {
        server = new WiFiServer(port);
        server->begin();
        server->setNoDelay(true);
        Serial.print("[LOG] TCP log server on port ");
        Serial.println(port);
    }

    static void startTask(uint32_t stack = 4096, UBaseType_t prio = 3, BaseType_t core = APP_CPU_NUM) {
        if (!queue) queue = xQueueCreate(kQueueDepth, sizeof(Msg));
        Serial.println("[LOG] queued network logging enabled from now on");
        xTaskCreatePinnedToCore(_taskEntry, "logger", stack, nullptr, prio, nullptr, core);
    }

    // printf-style (preferred; ensures line-atomic chunks you pass in)
    static void printf(const char* fmt, ...) {
        Msg m;
        va_list ap;
        va_start(ap, fmt);
        int n = vsnprintf(m.buf, sizeof(m.buf), fmt, ap);
        va_end(ap);
        if (n < 0) return;
        if (!queue) {
            // Not yet initialized, just print to serial.
            Serial.print(m.buf);
            return;
        }
        m.len = min(static_cast<int>(sizeof(m.buf)), n);
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
        m.len = (uint16_t)n;
        BaseType_t hpw = pdFALSE;
        xQueueSendFromISR(queue, &m, &hpw);
        portYIELD_FROM_ISR(hpw);
    }

private:
    static void _taskEntry(void*) {
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
                    _serialPrintf("[LOG] Client connected (%s)\n", c.remoteIP().toString().c_str());
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
                _serialPrintf("[LOG] Client disconnected\n");
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

        // Always write to Serial
        Serial.write(m.buf, m.len);

        if (!WiFi.isConnected()) {
            return;
        }

        if (mqttTopic != nullptr) {
            MQTT::publishAsync(mqttTopic, m.buf, m.len);
        }

        // Then to all clients (non-blocking; drop slow/broken)
        for (auto& c : clients) {
            if (!c || !c.connected()) continue;
            size_t off = 0;
            while (off < m.len) {
                int w = c.write(m.buf + off, m.len - off);
                if (w <= 0) {
                    c.stop();
                    break;
                } // drop if stalled
                off += w;
            }
        }
    }

    // Internal logging only to serial
    static void _serialPrintf(const char* fmt, ...) {
        char tmp[128];
        va_list ap;
        va_start(ap, fmt);
        int n = vsnprintf(tmp, sizeof(tmp), fmt, ap);
        va_end(ap);
        if (n > 0) Serial.write(reinterpret_cast<const uint8_t*>(tmp), (size_t)min(n, (int)sizeof(tmp)));
    }

private:
    static inline WiFiServer* server = nullptr;
    static inline WiFiClient clients[kMaxClients];
    static inline QueueHandle_t queue = nullptr;
    volatile static inline uint32_t droppedMessages = 0; // volatile is good enough for this
    static inline uint32_t lastDroppedMessages = 0;
    static inline const char* mqttTopic = nullptr;
};

// Convenience macros
#define LOGF(...)  Logger::printf(__VA_ARGS__)
#define LOGLN(s)   Logger::println(s)
#define LOG(s)     Logger::print(s)
