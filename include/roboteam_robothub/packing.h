#ifndef ROBOTHUB_SRC_PACKING_H_
#define ROBOTHUB_SRC_PACKING_H_

#include <array>
#include <cstdint>
#include <iostream>
#include <string>
#include <boost/optional.hpp>

namespace rtt {

boost::optional<std::array<uint8_t, 7>> createRobotPacket(int id, int robot_vel, int w,
                                        bool rot_cclockwise, int w_vel, uint8_t kick_force,
                                        bool do_kick, bool chip, bool forced,
                                        bool dribble_cclockwise, uint8_t dribble_vel);

template<unsigned int N>
std::string byteArrayToString(std::array<uint8_t, N> bytes) {
    std::string result = "";

    for (auto byte : bytes) {
        std::string byteStr = "";
        for (int i = 0; i < 8; i++) {
            if ((byte & (1 << i)) == (1 << i)) {
                byteStr = "1" + byteStr;
            } else {
                byteStr = "0" + byteStr;
            }
        }

        result += byteStr + "\n";
    }

    return result;
}

}

#endif // ROBOTHUB_SRC_PACKING_H_
