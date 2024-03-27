#ifndef RTT_LOGFILEWRITER_H
#define RTT_LOGFILEWRITER_H
#include <fstream>

#include "LogFileHeader.h"
#include "proto/State.pb.h"

namespace rtt {
class LogFileWriter {
   public:
    LogFileWriter() = default;
    /**
     * @brief Opens a LogFile with file_name. Returns true if this is successful, otherwise it returns false.
     */
    bool open(const std::string& file_name);

    /**
     * @brief If a file is open, closes it.
     */
    void close();
    /**
     * @brief Adds a single message at the end of the LogFile.
     * Returns true if done so succesfully.The given timestamp is checked: it must be the same or increasing!
     */
    bool addMessage(const logged_proto_type& message, logged_time_type timestamp);

   private:
    std::unique_ptr<std::ofstream> file;
    logged_time_type last_written_timestamp = 0;
    std::vector<char> serialization_buffer;
};
}  // namespace rtt

#endif  // RTT_LOGFILEWRITER_H
