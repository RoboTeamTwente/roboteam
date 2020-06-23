#ifndef ROBOTHUB_SRC_PACKING_H_
#define ROBOTHUB_SRC_PACKING_H_

#include <array>
#include <cstdint>
#include <iostream>
#include <string>
#include "roboteam_proto/RobotCommand.pb.h"
#include "roboteam_proto/RobotFeedback.pb.h"
#include "roboteam_proto/World.pb.h"

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
    int id : 8;
    bool xSensCalibrated : 1;
    bool batteryLow : 1;
    bool ballSensorWorking : 1;
    bool hasBall : 1;
    uint ballPosition : 4;
    bool genevaWorking : 1;
    int genevaState : 7;
    int rho : 11;
    int angle : 10;
    int theta : 11;
    bool hasLockedWheel : 1;
    uint signalStrength : 7;
};

// Software => Basestation
LowLevelRobotCommand createLowLevelRobotCommand(proto::RobotCommand const& command, proto::World worldOpt, bool isYellow);

std::shared_ptr<packed_protocol_message> createRobotPacket(LowLevelRobotCommand llrc);
std::shared_ptr<packed_protocol_message> createRobotPacket(proto::RobotCommand const& command, std::shared_ptr<proto::World> const& worldOpt = nullptr);

// Basestation => Software
LowLevelRobotFeedback createRobotFeedback(packed_robot_feedback bitsnbytes);

// Printing functions
void printRobotCommand(const proto::RobotCommand& cmd);
void printLowLevelRobotCommand(const LowLevelRobotCommand& llrc);
void printLowLevelRobotFeedback(const LowLevelRobotFeedback& llrf);
void printRobotFeedback(const proto::RobotFeedback& feedback);

bool validateRobotPacket(LowLevelRobotCommand llrc);
proto::RobotFeedback toRobotFeedback(LowLevelRobotFeedback feedback);

std::string byteToBinary(uint8_t const& byte);

template <unsigned int N>
std::string byteArrayToString(std::array<uint8_t, N> bytes) {
    std::string result = "";

    for (auto byte : bytes) {
        result += byteToBinary(byte) + "\n";
    }

    return result;
}

int constexpr PACKET_MAX_ROBOT_VEL = 8191;

int constexpr PACKET_MAX_ANG = 511;

int constexpr PACKET_MAX_W = 2047;

int constexpr PACKET_MAX_DRIBBLE_VEL = 7;

int constexpr PACKET_MAX_CAM_ROBOT_VEL = 8191;

int constexpr PACKET_MAX_CAM_ANG = 511;

int constexpr PACKET_MAX_CAM_W = 2047;

}  // namespace robothub
}  // namespace rtt

#endif  // ROBOTHUB_SRC_PACKING_H_
