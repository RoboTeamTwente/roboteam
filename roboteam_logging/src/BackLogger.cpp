#include "BackLogger.h"

#include "LogFileWriter.h"
rtt::BackLogger::BackLogger(rtt::logged_time_type interval_nanoseconds) : interval_duration{interval_nanoseconds} {}

void rtt::BackLogger::addMessage(const rtt::logged_proto_type& message, rtt::logged_time_type timestamp) {
    if (timestamp < queue.back().second) {
        std::cout << "State message has incorrect timestamp, cannot log properly!\n";
        return;
    }

    queue.emplace_back(message, timestamp);

    // can be more efficient for batches, but we usually only remove one message anyways
    while (queue.front().second + interval_duration < timestamp) {
        queue.pop_front();
    }
}

bool rtt::BackLogger::saveToFile(const std::string& file_name) {
    rtt::LogFileWriter writer;
    bool good = writer.open(file_name);
    if (!good) {
        return false;
    }
    for (const auto& [message, timestamp] : queue) {
        if (!writer.addMessage(message, timestamp)) {
            return false;
        }
    }
    writer.close();
    return true;
}
