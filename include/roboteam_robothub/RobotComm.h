#ifndef ROBOTHUB_INCLUDE_ROBOTCOMM_H_
#define ROBOTHUB_INCLUDE_ROBOTCOMM_H_

#include <array>
#include <cstdint>
#include <fstream>
#include <string>

namespace rtt {

class RobotComm {
public:
    RobotComm(std::string port = "/dev/ttyACM0");

private:
    std::fstream com;

    bool sendRobotPacket(
        int id, int robot_vel, int w,
        bool rot_cclockwise, int w_vel, uint8_t kick_force,
        bool do_kick, bool chip, bool forced,
        bool dribble_cclockwise, uint8_t dribble_vel);
    void send(std::array<uint8_t, 7> msg);
} ;

}

#endif
