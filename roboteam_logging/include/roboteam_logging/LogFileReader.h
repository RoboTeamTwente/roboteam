//
// Created by rolf on 28-05-22.
//

#ifndef RTT_LOGFILEREADER_H
#define RTT_LOGFILEREADER_H

#include <fstream>

#include "LogFileHeader.h"
namespace rtt {

    /**
     * @brief Class which reads the stored log files.
     * Prefer to keep backwards compatability for reading log files, so that we can keep using old log files.
     * For now, this simply indices the positions of all messages of a logfile when first opening it.
     * Then, we can quickly jump through the file for e.g. replaying it.
     */
    class LogFileReader {
    public:
        bool open(const std::string& file_name);
        void close();

        std::pair<logged_time_type, logged_proto_type> readFrame(std::size_t frame_number);
        std::pair<logged_time_type, logged_proto_type> readNext();

        [[nodiscard]] std::size_t fileMessageCount() const;
        void resetToStartOfFile();

    private:
        std::pair<logged_time_type, logged_proto_type> readPacket(long file_offset);
        bool indexFile();

        std::unique_ptr<std::ifstream> file;
        std::vector<long> index;  // index[packetnumber] returns the offset the packet is in
        int version = -1;
    };
}  // namespace rtt


#endif  // RTT_LOGFILEREADER_H
