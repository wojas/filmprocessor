#pragma once
#include <Arduino.h>
#include <PubSubClient.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <freertos/queue.h>


// ---------- Tunables (have defaults) ----------
#ifndef MQTT_TOPIC_MAX
#define MQTT_TOPIC_MAX        64
#endif
#ifndef MQTT_PAYLOAD_MAX
#define MQTT_PAYLOAD_MAX      1024
#endif
#ifndef MQTT_TX_QUEUE_DEPTH
#define MQTT_TX_QUEUE_DEPTH   16
#endif
#ifndef MQTT_RX_QUEUE_DEPTH
#define MQTT_RX_QUEUE_DEPTH   16
#endif
#ifndef MQTT_MAX_SUBS
#define MQTT_MAX_SUBS         16
#endif
#ifndef MQTT_CLIENTID_MAX
#define MQTT_CLIENTID_MAX     64
#endif
#ifndef MQTT_USER_MAX
#define MQTT_USER_MAX         64
#endif
#ifndef MQTT_PASS_MAX
#define MQTT_PASS_MAX         64
#endif


class MQTT {
public:
    // -------- Public RX view (no copies; points into static pool) --------
    struct RxView {
        const char* topic = nullptr;
        const uint8_t* payload = nullptr;
        uint16_t len = 0;
        uint16_t _idx = 0xFFFF; // opaque pool index to release
    };

    // -------- Lifecycle --------
    static void begin(Client& net,
                      const char* broker,
                      uint16_t port = 1883,
                      const char* clientId = nullptr,
                      const char* username = nullptr,
                      const char* password = nullptr) {
        // store network + broker
        _net = &net;
        _broker = broker;
        _port = port;

        _copyStr(_clientId(), MQTT_CLIENTID_MAX, clientId ? clientId : genClientId().c_str());
        _copyStr(_username(), MQTT_USER_MAX, username ? username : "");
        _copyStr(_password(), MQTT_PASS_MAX, password ? password : "");

        // PubSubClient setup
        _ps.setBufferSize(MQTT_PAYLOAD_MAX);
        _ps.setClient(*_net);
        _ps.setServer(_broker, _port);
        _ps.setCallback(&_onMessageThunk);

        // one-time init of queues & pools
        _ensureQueuesAndPools();
    }

    static void startTask(UBaseType_t prio = 1, uint32_t stack = 4096, BaseType_t core = APP_CPU_NUM) {
        if (_task) return;
        xTaskCreatePinnedToCore(&_taskEntry, "mqtt.loop", stack, nullptr, prio, &_task, core);
    }

    static void stopTask() {
        if (_task) {
            TaskHandle_t h = _task;
            _task = nullptr;
            vTaskDelete(h);
        }
    }

    // -------- Publish (enqueue; never blocks) --------
    static bool publishAsync(const char* topic, const uint8_t* data, size_t len) {
        if (!topic || !*topic || !data || len == 0) return false;
        if (len > MQTT_PAYLOAD_MAX) len = MQTT_PAYLOAD_MAX;

        uint16_t idx;
        if (xQueueReceive(_txFreeQ, &idx, 0) != pdTRUE) {
            Serial.println("(serial only) [MQTT] dropped message, no slot");
            return false; // no slot → drop
        }

        auto& n = _txPool()[idx];
        _copyStr(n.topic, MQTT_TOPIC_MAX, topic);
        n.len = static_cast<uint16_t>(len);
        memcpy(n.payload, data, len);
        n.retain = false;

        if (xQueueSend(_txQ, &idx, 0) != pdTRUE) {
            // should not fail often
            Serial.println("(serial only) [MQTT] publishAsync txQ send failed");
            xQueueSend(_txFreeQ, &idx, 0); // return slot
            return false;
        }
        return true;
    }

    static bool publishAsync(const char* topic, const char* s) {
        return s ? publishAsync(topic, reinterpret_cast<const uint8_t*>(s), strlen(s)) : false;
    }

    // -------- Subscribe (applied immediately if connected; remembered otherwise) --------
    static bool subscribe(const char* topic) {
        if (!topic || !*topic) return false;
        if (_ps.connected()) _ps.subscribe(topic);
        _rememberSub(topic);
        return true;
    }

    static bool unsubscribe(const char* topic) {
        if (!topic || !*topic) return false;
        if (_ps.connected()) _ps.unsubscribe(topic);
        _forgetSub(topic);
        return true;
    }

    // -------- Receive (non-blocking by default) --------
    // Fills a view pointing into RX pool; MUST call MQTT::free(view) after use.
    static bool recv(RxView& out, TickType_t wait = 0) {
        uint16_t idx;
        if (xQueueReceive(_rxQ, &idx, wait) != pdTRUE) return false;
        auto& n = _rxPool()[idx];
        out.topic = n.topic;
        out.payload = n.payload;
        out.len = n.len;
        out._idx = idx;
        return true;
    }

    static void free(RxView& v) {
        if (v._idx != 0xFFFF) {
            uint16_t idx = v._idx;
            v = RxView{};
            xQueueSend(_rxFreeQ, &idx, 0);
        }
    }

    // -------- Status --------
    static bool connected() { return _ps.connected(); }
    static const char* broker() { return _broker; }
    static uint16_t port() { return _port; }
    static const char* clientId() { return _clientId(); }

private:
    // -------- Node types (fixed-size) --------
    struct TxNode {
        char topic[MQTT_TOPIC_MAX];
        uint16_t len = 0;
        bool retain = false;
        uint8_t payload[MQTT_PAYLOAD_MAX];
    };

    struct RxNode {
        char topic[MQTT_TOPIC_MAX];
        uint16_t len = 0;
        uint8_t payload[MQTT_PAYLOAD_MAX];
    };

    
private:
        // Converted to C++17 inline static members (safe subset)
        inline static Client* _net = nullptr;
        inline static PubSubClient _ps;
        inline static const char* _broker = nullptr;
        inline static TaskHandle_t _task = nullptr;
        inline static QueueHandle_t _txQ = nullptr;
        inline static QueueHandle_t _txFreeQ = nullptr;
        inline static QueueHandle_t _rxQ = nullptr;
        inline static QueueHandle_t _rxFreeQ = nullptr;
        inline static TickType_t _backoff = pdMS_TO_TICKS(500);
        inline static uint16_t _port = 1883;
// -------- Function-local static singletons (header-only & portable) --------
static TxNode* _txPool() {
        static TxNode pool[MQTT_TX_QUEUE_DEPTH];
        return pool;
    }

    static RxNode* _rxPool() {
        static RxNode pool[MQTT_RX_QUEUE_DEPTH];
        return pool;
    }

    static char* _clientId() {
        static char buf[MQTT_CLIENTID_MAX] = {};
        return buf;
    }

    static char* _username() {
        static char buf[MQTT_USER_MAX] = {};
        return buf;
    }

    static char* _password() {
        static char buf[MQTT_PASS_MAX] = {};
        return buf;
    }

    static char** _subs() {
        static char* arr[MQTT_MAX_SUBS] = {};
        return arr;
    }
// -------- One-time init for queues and pool free-lists --------
    static void _ensureQueuesAndPools() {
        if (!_txQ) _txQ = xQueueCreate(MQTT_TX_QUEUE_DEPTH, sizeof(uint16_t));
        if (!_txFreeQ) _txFreeQ = xQueueCreate(MQTT_TX_QUEUE_DEPTH, sizeof(uint16_t));
        if (!_rxQ) _rxQ = xQueueCreate(MQTT_RX_QUEUE_DEPTH, sizeof(uint16_t));
        if (!_rxFreeQ) _rxFreeQ = xQueueCreate(MQTT_RX_QUEUE_DEPTH, sizeof(uint16_t));
        // populate free queues with all indices
        if (uxQueueMessagesWaiting(_txFreeQ) == 0) {
            for (uint16_t i = 0; i < MQTT_TX_QUEUE_DEPTH; i++)
                xQueueSend(_txFreeQ, &i, 0);
        }
        if (uxQueueMessagesWaiting(_rxFreeQ) == 0) {
            for (uint16_t i = 0; i < MQTT_RX_QUEUE_DEPTH; i++)
                xQueueSend(_rxFreeQ, &i, 0);
        }
    }

    // -------- Background task --------
    static void _taskEntry(void*) {
        for (;;) {
            if (!_ps.connected()) {
                _tryReconnect();
            } else {
                _ps.loop();
                _drainTx();
            }
            vTaskDelay(pdMS_TO_TICKS(10));
        }
    }

    static void _tryReconnect() {
        if (!_net) {
            Serial.println("[MQTT] reconnect: no net");
            vTaskDelay(pdMS_TO_TICKS(500));
            return;
        }
        _ps.setClient(*_net);
        _ps.setServer(_broker, _port);
        _ps.setCallback(&_onMessageThunk);

        bool ok = false;
        if (_username()[0]) {
            Serial.printf("[MQTT] connecting with auth (%s)...\n", _broker);
            ok = _ps.connect(_clientId(), _username(), _password());
        } else {
            Serial.printf("[MQTT] connecting without auth (%s)...\n", _broker);
            ok = _ps.connect(_clientId());
        }

        if (ok) {
            Serial.println("[MQTT] connected");
            _backoff = pdMS_TO_TICKS(500);
            _reapplySubs();
            _drainTx();
        } else {
            _backoff = std::min<TickType_t>(_backoff * 2, pdMS_TO_TICKS(5000));
            vTaskDelay(_backoff);
        }
    }

    static void _drainTx() {
        if (!_ps.connected()) return;
        uint16_t idx;
        while (xQueueReceive(_txQ, &idx, 0) == pdTRUE) {
            auto& n = _txPool()[idx];
            _ps.publish(n.topic, n.payload, n.len, n.retain);
            xQueueSend(_txFreeQ, &idx, 0); // return slot
        }
    }

    // -------- Inbound callback → copy into RX pool, enqueue index --------
    static void _onMessageThunk(char* topic, uint8_t* payload, unsigned int length) {
        uint16_t idx;
        if (xQueueReceive(_rxFreeQ, &idx, 0) != pdTRUE) return; // no space → drop
        auto& n = _rxPool()[idx];
        _copyStr(n.topic, MQTT_TOPIC_MAX, topic ? topic : "");
        if (length > MQTT_PAYLOAD_MAX) length = MQTT_PAYLOAD_MAX;
        n.len = static_cast<uint16_t>(length);
        if (length) memcpy(n.payload, payload, length);
        xQueueSend(_rxQ, &idx, 0);
    }

    // -------- Subscriptions (fixed table of C-strings) --------
    static void _rememberSub(const char* topic) {
        // already present?
        for (size_t i = 0; i < MQTT_MAX_SUBS; i++) {
            if (_subs()[i] && strncmp(_subs()[i], topic, MQTT_TOPIC_MAX) == 0) return;
        }
        // find empty slot
        for (size_t i = 0; i < MQTT_MAX_SUBS; i++) {
            if (!_subs()[i]) {
                _subs()[i] = _dupToStatic(topic);
                return;
            }
        }
    }

    static void _forgetSub(const char* topic) {
        for (size_t i = 0; i < MQTT_MAX_SUBS; i++) {
            if (_subs()[i] && strncmp(_subs()[i], topic, MQTT_TOPIC_MAX) == 0) {
                _subs()[i] = nullptr;
                return;
            }
        }
    }

    static void _reapplySubs() {
        for (size_t i = 0; i < MQTT_MAX_SUBS; i++) if (_subs()[i]) _ps.subscribe(_subs()[i]);
    }

    static char* _dupToStatic(const char* in) {
        static char store[MQTT_MAX_SUBS][MQTT_TOPIC_MAX] = {{0}};
        for (size_t i = 0; i < MQTT_MAX_SUBS; i++) {
            if (store[i][0] == 0) {
                _copyStr(store[i], MQTT_TOPIC_MAX, in);
                return store[i];
            }
        }
        return nullptr; // table full → silently ignore
    }

    // -------- Helpers --------
    static void _copyStr(char* dst, size_t cap, const char* src) {
        if (!dst || !cap) return;
        if (!src) {
            dst[0] = 0;
            return;
        }
        size_t n = strnlen(src, cap - 1);
        memcpy(dst, src, n);
        dst[n] = '\0';
    }

public:
    static String genClientId(const char* prefix = "esp32") {
        uint64_t mac = ESP.getEfuseMac();
        char id[48];
        snprintf(id, sizeof(id), "%s-%04X%04X-%lu",
                 prefix,
                 (uint16_t)(mac >> 32), (uint32_t)(mac & 0xFFFFFFFF),
                 (unsigned long)millis());
        return String(id);
    }
};
