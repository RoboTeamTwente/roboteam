#include <FileLogger.hpp>
#include <fstream>
#include <roboteam_utils/Print.h>

namespace rtt {

FileLogger::FileLogger(const std::string& path) {
    this->stream = std::make_unique<std::ofstream>(path.c_str(), std::ios_base::out | std::ios_base::app);
    RTT_DEBUG("Logging to: '", path, "'")
    if (this->stream == nullptr || !this->stream->is_open()) {
        throw std::runtime_error("Failed to open file log");
    }
}

FileLogger::~FileLogger() {
    if (this->stream != nullptr)
        this->stream->close();
}

void FileLogger::writeNewLine(const std::string& line) {
    *this->stream << line << std::endl;
}

void FileLogger::write(const std::string& text) {
    *this->stream << text;
}

bool FileLogger::flush() {
    this->stream->flush();
}

FailedToOpenFileException::FailedToOpenFileException(const std::string& _message) : message(_message){}

const char *FailedToOpenFileException::what() const noexcept {
    return this->message.c_str();
}

} // namespace rtt::robothub