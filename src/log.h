#ifndef RAYTRACER_LOG_H
#define RAYTRACER_LOG_H

#include <functional>
#include <sstream>
#include <string>

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
