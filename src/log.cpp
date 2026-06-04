#include "log.h"

#include <iostream>

namespace logging {

LogLine::~LogLine() {
    const char* tag = "";
    switch (level) {
        case Level::Info:  tag = "[INFO] ";  break;
        case Level::Warn:  tag = "[WARN] ";  break;
        case Level::Error: tag = "[ERROR] "; break;
    }
    std::cerr << tag << buffer.str() << '\n';
}

}  // namespace logging
