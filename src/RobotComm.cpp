#include "roboteam_robothub/packing.h"
#include "roboteam_robothub/RobotComm.h"

#include <array>

namespace rtt {

RobotComm::RobotComm(std::string port) : com{port} {}

bool RobotComm::sendRobotPacket(
        int id, int robot_vel, int w,
        bool rot_cclockwise, int w_vel, uint8_t kick_force,
        bool do_kick, bool chip, bool forced,
        bool dribble_cclockwise, uint8_t dribble_vel) {
    auto packet = createRobotPacket(
            id, robot_vel, w,
            rot_cclockwise, w_vel, kick_force,
            do_kick, chip, forced,
            dribble_cclockwise, dribble_vel);

    if (packet) {
        send(*packet);
    }
}

void RobotComm::send(std::array<uint8_t, 7> msg) {
    com.write(msg.data(), msg.size());
}

}
