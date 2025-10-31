#pragma once

#include <functional>
#include <string>
#include <vector>

/**
 * Utility class that buffers keypad input and dispatches handlers when patterns match.
 */
class InputMatch {
public:
    /**
     * Result passed to handlers describing the matched numeric payload, if any.
     */
    struct Result {
        bool has_number = false;
        int number = 0;
        bool leading_zero = false;
    };
    using Handler = std::function<void(const Result&)>;

    /**
     * Registers a pattern together with its handler.
     */
    void match(const char* pattern, Handler handler);
    /**
     * Consumes a single key press and dispatches any matching handler.
     */
    bool consume(char key);
    /**
     * Clears the current buffered input without invoking handlers.
     */
    void clear();
    /**
     * Returns the buffered input as a std::string.
     */
    std::string buffer() const;

private:
    /** Internal representation of a compiled pattern. */
    struct Rule {
        std::string literal;
        std::string prefix;
        std::string suffix;
        bool has_placeholder = false;
        Handler handler;
    };

    /** Returns true when the character represents a decimal digit. */
    static bool is_digit(char c);
    /** Attempts to match the current buffer against the provided rule. */
    bool apply_rule(const Rule& rule, Result& out) const;

    std::string buffer_;
    std::vector<Rule> rules_;
};
