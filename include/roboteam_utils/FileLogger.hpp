#pragma once

#include <fstream>
#include <string>

namespace rtt {

/* A small wrapper for ofstream, useful for logging to files.
 * It handles opening and closing of files.
 * Will append to previously written text in the file.
 * Throws FailedToOpenFileException if for example the
 * given folder path of the file does not exist.
 * Use the operator<< for writing to the file.
 * Saves at destruction. Use flush for intermediate saving. */
class FileLogger : public std::ofstream {
public:
    // Will create and open the file you specify. Eg. "log/LOG.txt"
    explicit FileLogger(const std::string& filePath);
    // Will close the file
    ~FileLogger();
};

class FailedToOpenFileException : public std::exception {
public:
    explicit FailedToOpenFileException(const std::string& message);
    [[nodiscard]] const char *what() const noexcept override;
private:
    const std::string message;
};

} // namespace rtt::robothub