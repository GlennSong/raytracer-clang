#ifndef RAYTRACER_LOG_H
#define RAYTRACER_LOG_H

#include <sstream>

namespace engine {
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
}  // namespace engine

#define LOG_INFO  ::engine::logging::LogLine(::engine::logging::Level::Info)
#define LOG_WARN  ::engine::logging::LogLine(::engine::logging::Level::Warn)
#define LOG_ERROR ::engine::logging::LogLine(::engine::logging::Level::Error)

#endif
