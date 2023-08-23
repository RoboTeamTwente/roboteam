#pragma once

#include <REM_BaseTypes.h>
#include <REM_Packet.h>

#include <fstream>
#include <mutex>

namespace rtt {

// This class is meant to contain all the thread-safe logging functionality of RobotHub
class RobotHubLogger {
   public:
    // Marple format is csv format. False for "human-readable" text file
    // Only moving is allowed
    RobotHubLogger(std::string filename);

    void writeREM(REM_PacketPayload* packet);
    void writeREM(REM_PacketPayload* packet, uint64_t timestamp);

   private:
    std::string filename;
    std::ofstream filestream;

    void write(uint8_t* byte_buffer, uint32_t size, uint64_t timestamp);
};

}  // namespace rtt