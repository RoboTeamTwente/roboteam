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

}

namespace rtt {

using packed_protocol_message = std::array<uint8_t, 7>;

boost::optional<packed_protocol_message> createRobotPacket(roboteam_msgs::RobotCommand const & command);
boost::optional<packed_protocol_message> createRobotPacket(int id, int robot_vel, int ang,
                                        bool rot_cclockwise, int w, uint8_t punt_power,
                                        bool do_kick, bool do_chip, bool forced,
                                        bool dribble_cclockwise, uint8_t dribble_vel);

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
int const PACKET_MAX_DRIBBLE_VEL = 7;

}

#endif // ROBOTHUB_SRC_PACKING_H_
