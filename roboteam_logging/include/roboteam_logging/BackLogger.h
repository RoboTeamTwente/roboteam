//
// Created by rolf on 06-06-22.
//

#ifndef RTT_BACKLOGGER_H
#define RTT_BACKLOGGER_H

#include "LogFileHeader.h"
#include <deque>

/**
 * A class which keeps a sliding window of the last x seconds of logged messages, and which optionally saves it to a file
 */

namespace rtt{
    class BackLogger {
    public:
        explicit BackLogger(logged_time_type interval_nanoseconds);
        void addMessage(const logged_proto_type& message, logged_time_type timestamp);
        bool saveToFile(const std::string& file_name);
    private:
        std::deque<std::pair<logged_proto_type,logged_time_type>> queue;

        logged_time_type interval_duration;
    };
}



#endif //RTT_BACKLOGGER_H
