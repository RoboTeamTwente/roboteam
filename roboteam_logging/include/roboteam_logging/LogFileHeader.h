//
// Created by rolf on 28-05-22.
//

#ifndef RTT_LOGFILEHEADER_H
#define RTT_LOGFILEHEADER_H

#include <proto/State.pb.h>

#include <cassert>
#include <cstring>
#include <limits>
namespace rtt {

    using logged_proto_type = proto::State;
    using logged_time_type = unsigned long long int;
    static constexpr logged_time_type INVALID_LOGGED_TIME = std::numeric_limits<logged_time_type>::max();

    static const char* DEFAULT_LOGFILE_HEADER_NAME = "RTT_LOG_FILE";
    static constexpr std::size_t LOGFILE_HEADER_NAME_SIZE = 12;  // The number of characters used in the header.
    // Any characters more are not written, and any characters less is a bug! Note that the null terminating character takes up one byte as well!


    static constexpr int LOGFILE_VERSION = 0;
    // Every log file has a header on top which is always populated with the name ("RTT_LOG_FILE") and a version.
    struct LogFileHeader {
        LogFileHeader()
            : name(),
              version(LOGFILE_VERSION) {
            strncpy(name, DEFAULT_LOGFILE_HEADER_NAME, LOGFILE_HEADER_NAME_SIZE);
        }
        char name[LOGFILE_HEADER_NAME_SIZE];
        int version;
    };
    // Before the actual content of each message,
    // we store a timestamp and its size: this way we can easily jump through a logfile without needing to deserialize all messages.
    struct LogDataHeader {
        unsigned long long int timestamp;
        std::size_t message_size;
    };
}  // namespace rtt

#endif  // RTT_LOGFILEHEADER_H
