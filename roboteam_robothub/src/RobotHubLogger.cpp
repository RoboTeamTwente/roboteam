#include <REM_BaseTypes.h>
#include <REM_Packet.h>
#include <REM_RobotCommand.h>
#include <REM_RobotFeedback.h>
#include <roboteam_utils/Time.h>

#include <RobotHubLogger.hpp>
#include <roboteam_utils/Format.hpp>
#include <thread>

namespace rtt {

RobotHubLogger::RobotHubLogger(std::string filename) : filename(filename) {
    filestream.open(filename, std::ios::out | std::ios::binary);

    if (!filestream.is_open()) {
        throw std::runtime_error("Could not open log file " + filename);
    }
}

void RobotHubLogger::writeREM(REM_PacketPayload* packet) {
    uint64_t timestamp = static_cast<uint64_t>(Time::now().asMilliSeconds());
    writeREM(packet, timestamp);
}

void RobotHubLogger::writeREM(REM_PacketPayload* packetPayload, uint64_t timestamp) { write((uint8_t*)packetPayload, REM_Packet_get_payloadSize(packetPayload), timestamp); }

void RobotHubLogger::write(uint8_t* byte_buffer, uint32_t size, uint64_t timestamp) {
    // Write the uint64_t timestamp to the file using reinterpret_casts
    // TODO Check if this is written in little endian
    filestream.write(reinterpret_cast<char*>(&timestamp), sizeof(uint64_t));
    // Write the packet
    filestream.write((char*)byte_buffer, size);
}

}  // namespace rtt

/*int main(int argc, char **argv) {

    std::string timeString = Time::getDate('-') + "_" + Time::getTime('-');
    std::string filename = "robothub_log_" + timeString + ".rembin";

    rtt::RobotHubLogger logger("logfile.rembin");

    REM_Packet packet;
    packet.packetType = REM_PACKET_TYPE_REM_PACKET;
    packet.fromPC = 1;
    packet.toPC = 1;
    packet.remVersion = REM_LOCAL_VERSION;
    packet.payloadSize = REM_PACKET_SIZE_REM_PACKET;

    REM_PacketPayload payload;
    encodeREM_Packet(&payload, &packet);
    logger.writeREM(&payload);

    REM_RobotCommand command;
    command.packetType = REM_PACKET_TYPE_REM_ROBOT_COMMAND;
    command.fromPC = 1;
    command.toPC = 1;
    command.remVersion = REM_LOCAL_VERSION;
    command.payloadSize = REM_PACKET_SIZE_REM_ROBOT_COMMAND;

    REM_RobotCommandPayload commandPayload;
    encodeREM_RobotCommand(&commandPayload, &command);

    REM_RobotFeedback feedback;
    feedback.packetType = REM_PACKET_TYPE_REM_ROBOT_FEEDBACK;
    feedback.fromPC = 1;
    feedback.toPC = 1;
    feedback.remVersion = REM_LOCAL_VERSION;
    feedback.payloadSize = REM_PACKET_SIZE_REM_ROBOT_FEEDBACK;

    REM_RobotFeedbackPayload feedbackPayload;
    encodeREM_RobotFeedback(&feedbackPayload, &feedback);

    for (int i = 0; i < 100; i++) {
        logger.writeREM(&payload);

        REM_RobotCommand_set_messageId(&commandPayload, i);
        REM_RobotFeedback_set_messageId(&feedbackPayload, i);

        logger.writeREM((REM_PacketPayload*) &commandPayload);
        logger.writeREM((REM_PacketPayload*) &feedbackPayload);

        // Sleep between 1 and 5 milliseconds
        std::this_thread::sleep_for(std::chrono::milliseconds(1 + (rand() % 5)));

    }

    return 0;
}*/