#ifndef RAYTRACER_LOG_H
#define RAYTRACER_LOG_H

#include <cstdarg>
#include <cstdio>
#include <functional>
#include <sstream>
#include <string>
#include <vector>

namespace engine {
namespace logging {

enum class Level { Info, Warn, Error };

// Optional secondary sink (e.g. the editor's console dock), called for every
// line after stderr. May fire from worker threads (physics jobs log), so the
// sink must be thread-safe; calls are serialized by the log's own mutex.
// Pass nullptr to clear.
void setSink(std::function<void(Level, const std::string&)> sink);

// Accumulates a message and flushes it as one tagged line to stderr on
// destruction, so `LOG_ERROR << a << b;` emits a single line at end of
// statement.
class LogLine {
public:
    explicit LogLine(Level level) : level(level) {}
    ~LogLine();

    template <typename T>
    LogLine& operator<<(const T& value) {
        buffer << value;
        return *this;
    }

    // printf-style sink: `LOG_ERROR("x=%d", x);`. Provided alongside operator<<
    // so subsystems written against a format-string logger (e.g. the Vulkan
    // backend) compile without rewriting every call site. A bare string with a
    // literal '%' must escape it as "%%", same as printf.
    LogLine& operator()(const char* fmt, ...)
#if defined(__GNUC__) || defined(__clang__)
        __attribute__((format(printf, 2, 3)))
#endif
    {
        va_list args;
        va_start(args, fmt);
        va_list measure;
        va_copy(measure, args);
        int n = std::vsnprintf(nullptr, 0, fmt, measure);
        va_end(measure);
        if (n > 0) {
            std::vector<char> tmp(static_cast<size_t>(n) + 1);
            std::vsnprintf(tmp.data(), tmp.size(), fmt, args);
            buffer.write(tmp.data(), n);
        }
        va_end(args);
        return *this;
    }

private:
    Level level;
    std::ostringstream buffer;
};

}  // namespace logging
}  // namespace engine

// Parenthesized so `LOG_X("fmt", ...)` parses as a call to LogLine::operator()
// rather than a declaration (most-vexing-parse). `LOG_X << ...` is unaffected.
#define LOG_INFO  (::engine::logging::LogLine(::engine::logging::Level::Info))
#define LOG_WARN  (::engine::logging::LogLine(::engine::logging::Level::Warn))
#define LOG_ERROR (::engine::logging::LogLine(::engine::logging::Level::Error))

#endif
