#include "stp/skills/Kick.h"

#include "roboteam_utils/Print.h"
#include "utilities/Constants.h"

namespace rtt::ai::stp::skill {

Status Kick::onUpdate(const StpInfo &info) noexcept {
    auto robot = info.getRobot().value();

    // Clamp and set kick velocity
    float kickVelocity = std::clamp(info.getKickChipVelocity(), constants::MIN_KICK_POWER, constants::MAX_KICK_POWER);
    command.kickType = KickType::KICK;
    command.kickSpeed = kickVelocity;

    command.dribblerOn = info.getDribblerOn();

    // Set yaw command
    command.yaw = robot->getYaw();

    command.waitForBall = false;

    // set command ID
    command.id = robot->getId();

    // forward the generated command to the ControlModule, for checking and limiting
    forwardRobotCommand();

    if (!robot->hasBall() && info.getBall()->get()->velocity.length() > constants::BALL_GOT_SHOT_LIMIT) {
        return Status::Success;
    }
    return Status::Running;
}

const char *Kick::getName() { return "Kick"; }

}  // namespace rtt::ai::stp::skill