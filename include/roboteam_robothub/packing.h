#ifndef ROBOTHUB_SRC_PACKING_H_
#define ROBOTHUB_SRC_PACKING_H_

#include <array>
#include <cstdint>
#include <iostream>
#include <string>
#include <boost/optional.hpp>

#include <ros/message_forward.h>

namespace roboteam_msgs {

ROS_DECLARE_MESSAGE(RobotCommand);
ROS_DECLARE_MESSAGE(World)

}

namespace rtt {

struct LowLevelRobotCommand {
    int id;
    int robot_vel;
    int ang;
    bool rot_cclockwise;
    int w;
    // Indicates power for both kicking & chipping
    uint8_t punt_power;
    bool cam_data_on;
    bool do_kick;
    bool do_chip;
    bool forced;
    bool dribble_cclockwise;
    uint8_t dribble_vel;

    // Cam data
    int32_t cam_robot_vel;
    int32_t cam_ang;
    bool cam_rot_cclockwise;
    int32_t cam_w;
} ;


using packed_protocol_message = std::array<uint8_t, 12>;

LowLevelRobotCommand createLowLevelRobotCommand(roboteam_msgs::RobotCommand const & command, boost::optional<roboteam_msgs::World const &> worldOpt = boost::none);
boost::optional<packed_protocol_message> createRobotPacket(LowLevelRobotCommand llrc);
boost::optional<packed_protocol_message> createRobotPacket(roboteam_msgs::RobotCommand const & command);

boost::optional<packed_protocol_message> createRobotPacket(int32_t id, int32_t robot_vel, int32_t ang,
                                                         bool rot_cclockwise, int32_t w, uint8_t punt_power,
                                                         bool do_kick, bool do_chip, bool forced,
                                                         bool dribble_cclockwise, uint8_t dribble_vel);

boost::optional<packed_protocol_message> createRobotPacket(int32_t id, int32_t robot_vel, int32_t ang,
                                                         bool rot_cclockwise, int32_t w, uint8_t punt_power,
                                                         bool cam_data_on,
                                                         bool do_kick, bool do_chip, bool forced,
                                                         bool dribble_cclockwise, uint8_t dribble_vel,
                                                         int32_t cam_robot_vel, int32_t cam_ang,
                                                         bool cam_rot_cclockwise, int32_t cam_w);

std::string byteToBinary(uint8_t byte);

template<unsigned int N>
std::string byteArrayToString(std::array<uint8_t, N> bytes) {
    std::string result = "";

    for (auto byte : bytes) {
        result += byteToBinary(byte) + "\n";
    }

    return result;
}

int const PACKET_MAX_ROBOT_VEL = 8191;
int const PACKET_MAX_ANG = 511;
int const PACKET_MAX_W = 2047;
int const PACKET_MAX_DRIBBLE_VEL = 4;

int const PACKET_MAX_CAM_ROBOT_VEL = 8191;
int const PACKET_MAX_CAM_ANG = 511;
int const PACKET_MAX_CAM_W = 2047;

}

bool operator==(const rtt::LowLevelRobotCommand& lhs, const rtt::LowLevelRobotCommand& rhs);
bool operator!=(const rtt::LowLevelRobotCommand& lhs, const rtt::LowLevelRobotCommand& rhs);

#endif // ROBOTHUB_SRC_PACKING_H_
