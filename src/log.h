#ifndef RAYTRACER_LOG_H
#define RAYTRACER_LOG_H

#include <sstream>

namespace logging {

enum class Level { Info, Warn, Error };

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

private:
    Level level;
    std::ostringstream buffer;
};

}  // namespace logging

#define LOG_INFO  ::logging::LogLine(::logging::Level::Info)
#define LOG_WARN  ::logging::LogLine(::logging::Level::Warn)
#define LOG_ERROR ::logging::LogLine(::logging::Level::Error)

#endif
