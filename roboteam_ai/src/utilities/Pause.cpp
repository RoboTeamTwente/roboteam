//
// Created by baris on 15-2-19.
//

#include <roboteam_utils/RobotCommands.hpp>

#include "utilities/IOManager.h"
#include "world/World.hpp"

namespace rtt::ai {

std::atomic<bool> Pause::state = false;

bool Pause::isPaused() { return state; }
void Pause::pause(std::optional<rtt::world::view::WorldDataView> data) {
    state = true;
    if (!data) {
        return;
    }

    auto us = data->getUs();
    std::vector<rtt::RobotCommand> commands;
    for (const auto& robot : us) {
        rtt::RobotCommand cmd = {};
        cmd.id = robot->getId();
        cmd.velocity.x = 0;
        cmd.velocity.y = 0;
        cmd.useAngularVelocity = false;
        cmd.targetAngle = robot->getAngle();
        commands.push_back(std::move(cmd));
    }
    io::io.publishAllRobotCommands(commands);
}
void Pause::resume() { state = false; }

}  // namespace rtt::ai