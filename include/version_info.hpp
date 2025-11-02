#pragma once

#ifdef ARDUINO
#include <Arduino.h>
#else
#include "Arduino.h"
#endif

#if defined(__has_include)
#if __has_include("version_info_data.hpp")
#include "version_info_data.hpp"
#endif
#else
#include "version_info_data.hpp"
#endif

#ifndef VERSION_INFO_HAVE_GENERATED
namespace version_generated {
inline constexpr bool kGitIsTag = false;
inline constexpr bool kGitDirty = false;
inline constexpr char kGitHashStr[] = "unknown";
inline constexpr char kGitRefStr[] = "unknown";
inline constexpr char kGitCommitIso[] = "unknown";
inline constexpr char kGitScreenLine0[] = "git: unknown     ";
inline constexpr char kGitScreenLine1[] = "unknown          ";
inline constexpr char kGitDateLine[] = "Date: unknown    ";
inline constexpr char kGitTimeLine[] = "Time: unknown    ";
} // namespace version_generated
#endif

namespace version_info {

inline bool is_dirty() {
    return version_generated::kGitDirty;
}

inline String hash() {
    return String(version_generated::kGitHashStr);
}

inline String ref() {
    return String(version_generated::kGitRefStr);
}

inline bool is_tag() {
    return version_generated::kGitIsTag;
}

inline String commit_timestamp_iso() {
    return String(version_generated::kGitCommitIso);
}

inline String screen_git_line0() {
    return String(version_generated::kGitScreenLine0);
}

inline String screen_git_line1() {
    return String(version_generated::kGitScreenLine1);
}

inline String commit_date_line() {
    return String(version_generated::kGitDateLine);
}

inline String commit_time_line() {
    return String(version_generated::kGitTimeLine);
}

} // namespace version_info
