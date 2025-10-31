#pragma once

#ifdef ARDUINO
#include_next <Arduino.h>
#else

#include <algorithm>
#include <chrono>
#include <cstdint>
#include <cstring>
#include <string>
#include <thread>

using uint8_t = std::uint8_t;
using uint16_t = std::uint16_t;
using uint32_t = std::uint32_t;

inline unsigned long millis() {
    static auto start = std::chrono::steady_clock::now();
    auto now = std::chrono::steady_clock::now();
    return static_cast<unsigned long>(
        std::chrono::duration_cast<std::chrono::milliseconds>(now - start).count());
}

inline void delay(unsigned long ms) {
    std::this_thread::sleep_for(std::chrono::milliseconds(ms));
}

#ifndef F
#define F(x) x
#endif

class String {
public:
    String() = default;
    String(const char* s) : data_(s ? s : "") {}
    String(const char* s, size_t len) : data_(s ? std::string(s, len) : std::string()) {}
    String(const std::string& s) : data_(s) {}
    String(char c) : data_(1, c) {}

    size_t length() const { return data_.size(); }
    bool isEmpty() const { return data_.empty(); }
    const char* c_str() const { return data_.c_str(); }
    void reserve(size_t n) { data_.reserve(n); }
    void setCharAt(size_t idx, char c) {
        if (idx < data_.size()) {
            data_[idx] = c;
        }
    }
    char operator[](size_t idx) const { return data_[idx]; }
    char& operator[](size_t idx) { return data_[idx]; }
    String substring(size_t start) const {
        if (start >= data_.size()) {
            return String("");
        }
        return String(data_.substr(start));
    }
    String substring(size_t start, size_t end) const {
        if (start >= data_.size() || end <= start) {
            return String("");
        }
        if (end > data_.size()) {
            end = data_.size();
        }
        return String(data_.substr(start, end - start));
    }
    String& operator=(const char* s) {
        data_ = s ? s : "";
        return *this;
    }
    String& operator=(const String&) = default;
    bool operator==(const char* s) const { return data_ == (s ? s : ""); }
    operator std::string() const { return data_; }
    String& operator+=(const String& other) {
        data_ += other.data_;
        return *this;
    }
    String& operator+=(char c) {
        data_ += c;
        return *this;
    }
    friend String operator+(const String& lhs, const String& rhs) {
        return String(lhs.data_ + rhs.data_);
    }
    friend String operator+(const String& lhs, const char* rhs) {
        return String(lhs.data_ + (rhs ? std::string(rhs) : std::string()));
    }
    friend String operator+(const char* lhs, const String& rhs) {
        return String((lhs ? std::string(lhs) : std::string()) + rhs.data_);
    }

private:
    std::string data_;
};

#endif // ARDUINO
