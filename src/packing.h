#ifndef ROBOTHUB_SRC_PACKING_H_
#define ROBOTHUB_SRC_PACKING_H_

#include <array>
#include <cstdint>
#include <iostream>
#include <string>
#include <ros/message_forward.h>
#include "roboteam_msgs/World.h"

using packed_protocol_message = std::array<uint8_t, 10>;
using packed_robot_feedback = std::array<uint8_t, 23>;
using boring_ack = std::array<uint8_t, 2>;

namespace roboteam_msgs {
    ROS_DECLARE_MESSAGE(RobotCommand);
    ROS_DECLARE_MESSAGE(RobotFeedback);
}

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

    bool wheelLeftFront;
    bool wheelRightFront;
    bool wheelLeftBack;
    bool wheelRightBack;

    bool genevaDriveState;
    bool batteryState;

    int position_x;
    int position_y;

    int rho;
    int theta;
    int orientation;

    int angularVelocity;
    int ballSensor;

    float acceleration_x;
    float acceleration_y;
    float velocity_angular;
};

// Software => Basestation
LowLevelRobotCommand createLowLevelRobotCommand(
        roboteam_msgs::RobotCommand const& command, std::shared_ptr<roboteam_msgs::World> const& worldOpt = nullptr
);

std::shared_ptr<packed_protocol_message> createRobotPacket(LowLevelRobotCommand llrc);
std::shared_ptr<packed_protocol_message> createRobotPacket(
        roboteam_msgs::RobotCommand const& command, std::shared_ptr<roboteam_msgs::World> const& worldOpt = nullptr
);

// Basestation => Software
LowLevelRobotFeedback createRobotFeedback(packed_robot_feedback bitsnbytes);

// Printing functions
void printRobotCommand(const roboteam_msgs::RobotCommand& cmd);
void printLowLevelRobotCommand(const LowLevelRobotCommand& llrc);
void printLowLevelRobotFeedback(const LowLevelRobotFeedback& llrf);
void printRobotFeedback(const roboteam_msgs::RobotFeedback& feedback);

bool validateRobotPacket(LowLevelRobotCommand llrc);
roboteam_msgs::RobotFeedback toRobotFeedback(LowLevelRobotFeedback feedback);

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
