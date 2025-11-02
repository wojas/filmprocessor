#pragma once

#ifdef ARDUINO
#include <Arduino.h>
#else
#include "Arduino.h"
#endif

#ifndef GIT_HASH
#define GIT_HASH "unknown"
#endif

#ifndef GIT_REF
#define GIT_REF "unknown"
#endif

#ifndef GIT_COMMIT_TIME
#define GIT_COMMIT_TIME "unknown"
#endif

#ifndef GIT_IS_TAG
#define GIT_IS_TAG 0
#endif

#ifndef GIT_DIRTY
#define GIT_DIRTY 0
#endif

namespace version_info {

#define VERSION_INFO_STRINGIZE_INNER(x) #x
#define VERSION_INFO_STRINGIZE(x) VERSION_INFO_STRINGIZE_INNER(x)

inline String macro_string(const char* raw) {
    String result = String(raw);
    int len = result.length();
    if (len >= 2 && result[0] == '"' && result[len - 1] == '"') {
        result = result.substring(1, len - 1);
    }
    return result;
}

inline bool is_dirty() {
    return GIT_DIRTY != 0;
}

inline String hash() {
    String value = macro_string(VERSION_INFO_STRINGIZE(GIT_HASH));
    if (is_dirty()) {
        int len = value.length();
        if (len == 0 || value[len - 1] != '+') {
            value += '+';
        }
    }
    return value;
}

inline String ref() {
    return macro_string(VERSION_INFO_STRINGIZE(GIT_REF));
}

inline bool is_tag() {
    return static_cast<bool>(GIT_IS_TAG);
}

inline String commit_timestamp_iso() {
    return macro_string(VERSION_INFO_STRINGIZE(GIT_COMMIT_TIME));
}

inline String screen_git_line0() {
    String line = "git: ";
    line += hash();
    if (line.length() > 16) {
        line = line.substring(0, 16);
    }
    while (line.length() < 16) {
        line += ' ';
    }
    return line;
}

inline String screen_git_line1() {
    String label = ref();
    if (label.length() > 16) {
        label = label.substring(0, 16);
    }
    while (label.length() < 16) {
        label += ' ';
    }
    return label;
}

inline String commit_date_line() {
    String iso = commit_timestamp_iso();
    String datePart = iso.length() >= 10 ? iso.substring(0, 10) : iso;
    String line = "Date: " + datePart;
    if (line.length() > 16) {
        line = line.substring(0, 16);
    }
    while (line.length() < 16) {
        line += ' ';
    }
    return line;
}

inline String commit_time_line() {
    String iso = commit_timestamp_iso();
    String timePart;
    if (iso.length() >= 19) {
        timePart = iso.substring(11, 19);
    } else if (iso.length() > 11) {
        timePart = iso.substring(11);
    } else {
        timePart = iso;
    }
    String line = "Time: " + timePart;
    if (line.length() > 16) {
        line = line.substring(0, 16);
    }
    while (line.length() < 16) {
        line += ' ';
    }
    return line;
}

} // namespace version_info

#undef VERSION_INFO_STRINGIZE
#undef VERSION_INFO_STRINGIZE_INNER

#undef VERSION_INFO_STRINGIZE
#undef VERSION_INFO_STRINGIZE_INNER
