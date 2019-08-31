#ifndef ROBOTHUB_SRC_PACKING_H_
#define ROBOTHUB_SRC_PACKING_H_

#include <array>
#include <cstdint>
#include <iostream>
#include <string>
#include <RobotCommand.pb.h>
#include <RobotFeedback.pb.h>
#include <World.pb.h>

using packed_protocol_message = std::array<uint8_t, 10>;
using packed_robot_feedback = std::array<uint8_t, 8>;
using boring_ack = std::array<uint8_t, 2>;

namespace rtt {
namespace robothub {

// The comments might not be fully correct.

struct LowLevelRobotCommand {
    int id;
    int rho;
    int theta;
    bool driving_reference;
    bool use_cam_info;
    bool use_angle;
    int velocity_angular;
    bool debug_info;
    bool do_kick;
    bool do_chip;
    bool kick_chip_forced;
    int kick_chip_power;
    int velocity_dribbler;
    int geneva_drive_state;
    int cam_position_x;
    int cam_position_y;
    int cam_rotation;
};

struct LowLevelRobotFeedback {
    int id;
    bool xSensCalibrated;
    bool batteryLow;
    bool ballSensorWorking;
    bool hasBall;
    int ballPosition;
    bool genevaWorking;
    int genevaState;
    int rho;
    int angle;
    int theta;
    bool hasLockedWheel;
    int signalStrength;
};

// Software => Basestation
LowLevelRobotCommand createLowLevelRobotCommand(
        roboteam_proto::RobotCommand const& command, roboteam_proto::World worldOpt
);

std::shared_ptr<packed_protocol_message> createRobotPacket(LowLevelRobotCommand llrc);
std::shared_ptr<packed_protocol_message> createRobotPacket(
    roboteam_proto::RobotCommand const& command, std::shared_ptr<roboteam_proto::World> const& worldOpt = nullptr
);

// Basestation => Software
LowLevelRobotFeedback createRobotFeedback(packed_robot_feedback bitsnbytes);

// Printing functions
void printRobotCommand(const roboteam_proto::RobotCommand& cmd);
void printLowLevelRobotCommand(const LowLevelRobotCommand& llrc);
void printLowLevelRobotFeedback(const LowLevelRobotFeedback& llrf);
void printRobotFeedback(const roboteam_proto::RobotFeedback& feedback);

bool validateRobotPacket(LowLevelRobotCommand llrc);
roboteam_proto::RobotFeedback toRobotFeedback(LowLevelRobotFeedback feedback);

std::string byteToBinary(uint8_t const& byte);

template<unsigned int N>
std::string byteArrayToString(std::array<uint8_t, N> bytes)
{
    std::string result = "";

    for (auto byte : bytes) {
        result += byteToBinary(byte)+"\n";
    }

    return result;
}

int const PACKET_MAX_ROBOT_VEL = 8191;

int const PACKET_MAX_ANG = 511;

int const PACKET_MAX_W = 2047;

int const PACKET_MAX_DRIBBLE_VEL = 7;

int const PACKET_MAX_CAM_ROBOT_VEL = 8191;

int const PACKET_MAX_CAM_ANG = 511;

int const PACKET_MAX_CAM_W = 2047;

} // robothub
} // rtt

#endif // ROBOTHUB_SRC_PACKING_H_
