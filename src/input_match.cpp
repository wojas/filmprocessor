#include "input_match.hpp"

#include <utility>

void InputMatch::match(const char* pattern, Handler handler) {
    if (pattern == nullptr || !handler) {
        return;
    }
    Rule rule;
    rule.handler = std::move(handler);
    std::string pat(pattern);
    const std::size_t first_n = pat.find('n');
    if (first_n == std::string::npos) {
        rule.literal = pat;
    } else {
        rule.has_placeholder = true;
        rule.prefix = pat.substr(0, first_n);
        std::size_t idx = first_n;
        while (idx < pat.size() && pat[idx] == 'n') {
            ++idx;
        }
        for (std::size_t i = idx; i < pat.size(); ++i) {
            if (pat[i] == 'n') {
                // Multiple placeholders are not supported.
                return;
            }
        }
        rule.suffix = pat.substr(idx);
    }
    rules_.push_back(std::move(rule));
}

bool InputMatch::consume(char key) {
    if (!buffer_.empty() && key == '*') {
        buffer_.clear();
        return true;
    }
    if (buffer_.empty() && key == '*') {
        buffer_.push_back(key);
    } else {
        buffer_.push_back(key);
    }
    for (const auto& rule : rules_) {
        Result result;
        if (apply_rule(rule, result)) {
            rule.handler(result);
            buffer_.clear();
            return true;
        }
    }
    return false;
}

void InputMatch::clear() {
    buffer_.clear();
}

String InputMatch::buffer() const {
    return String(buffer_.c_str());
}

bool InputMatch::is_digit(char c) {
    return c >= '0' && c <= '9';
}

bool InputMatch::apply_rule(const Rule& rule, Result& out) const {
    if (rule.has_placeholder) {
        if (buffer_.size() < rule.prefix.size() + rule.suffix.size() + 1) {
            return false;
        }
        if (buffer_.compare(0, rule.prefix.size(), rule.prefix) != 0) {
            return false;
        }
        if (buffer_.compare(buffer_.size() - rule.suffix.size(), rule.suffix.size(), rule.suffix) != 0) {
            return false;
        }
        const std::size_t digits_len = buffer_.size() - rule.prefix.size() - rule.suffix.size();
        if (digits_len == 0) {
            return false;
        }
        std::string digits = buffer_.substr(rule.prefix.size(), digits_len);
        for (char c : digits) {
            if (!is_digit(c)) {
                return false;
            }
        }
        int value = 0;
        for (char c : digits) {
            value = value * 10 + (c - '0');
        }
        out.has_number = true;
        out.number = value;
        out.leading_zero = digits.size() > 1 && digits.front() == '0';
        return true;
    }
    if (buffer_ == rule.literal) {
        out.has_number = false;
        out.number = 0;
        out.leading_zero = false;
        return true;
    }
    return false;
}
