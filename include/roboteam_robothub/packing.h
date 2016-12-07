#ifndef ROBOTHUB_SRC_PACKING_H_
#define ROBOTHUB_SRC_PACKING_H_

#include <array>
#include <cstdint>
#include <iostream>
#include <string>
#include <boost/optional.hpp>

namespace rtt {

using packed_protocol_message = std::array<uint8_t, 7>;

boost::optional<packed_protocol_message> createRobotPacket(int id, int robot_vel, int w,
                                        bool rot_cclockwise, int w_vel, uint8_t kick_force,
                                        bool do_kick, bool chip, bool forced,
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

}

#endif // ROBOTHUB_SRC_PACKING_H_
