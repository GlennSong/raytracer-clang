#include "log.h"

#include <iostream>
#include <mutex>

namespace engine {
namespace logging {

namespace {
std::mutex sinkMutex;
std::function<void(Level, const std::string&)> sink;
}  // namespace

void setSink(std::function<void(Level, const std::string&)> newSink) {
    std::lock_guard<std::mutex> lock(sinkMutex);
    sink = std::move(newSink);
}

LogLine::~LogLine() {
    const char* tag = "";
    switch (level) {
        case Level::Info:  tag = "[INFO] ";  break;
        case Level::Warn:  tag = "[WARN] ";  break;
        case Level::Error: tag = "[ERROR] "; break;
    }
    std::cerr << tag << buffer.str() << '\n';

    std::lock_guard<std::mutex> lock(sinkMutex);
    if (sink) sink(level, buffer.str());
}

}  // namespace logging
}  // namespace engine
