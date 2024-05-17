#include "stp/skills/Kick.h"

#include "roboteam_utils/Print.h"
#include "stp/constants/ControlConstants.h"

namespace rtt::ai::stp::skill {

Status Kick::onUpdate(const StpInfo &info) noexcept {
    auto robot = info.getRobot().value();

    // Clamp and set kick velocity
    float kickVelocity = std::clamp(info.getKickChipVelocity(), control_constants::MIN_KICK_POWER, control_constants::MAX_KICK_POWER);
    command.kickType = KickType::KICK;
    command.kickSpeed = kickVelocity;

    // Clamp and set dribbler speed
    int targetDribblerPercentage = std::clamp(info.getDribblerSpeed(), 0, 100);
    double targetDribblerSpeed = targetDribblerPercentage / 100.0 * stp::control_constants::MAX_DRIBBLER_CMD;
    command.dribblerSpeed = targetDribblerSpeed;

    // Set angle command
    command.targetAngle = robot->getAngle();

    command.waitForBall = false;

    // set command ID
    command.id = robot->getId();

    // forward the generated command to the ControlModule, for checking and limiting
    forwardRobotCommand();

    if (!robot->hasBall() && info.getBall()->get()->velocity.length() > control_constants::BALL_GOT_SHOT_LIMIT) {
        return Status::Success;
    }
    return Status::Running;
}

const char *Kick::getName() { return "Kick"; }

}  // namespace rtt::ai::stp::skill