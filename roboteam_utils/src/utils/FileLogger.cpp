#include <FileLogger.hpp>
#include <fstream>
#include <roboteam_utils/Print.h>

namespace rtt {

FileLogger::FileLogger(const std::string& path) {
    this->open(path, std::ios_base::out | std::ios_base::app);
    if (!this->is_open()) throw FailedToOpenFileException("Failed to open log file: '" + path + "'");
    RTT_DEBUG("Logging to: '", path, "'")
}

FileLogger::~FileLogger() {
    this->close();
}

FailedToOpenFileException::FailedToOpenFileException(const std::string& _message) : message(_message){}

const char *FailedToOpenFileException::what() const noexcept {
    return this->message.c_str();
}

} // namespace rtt::robothub