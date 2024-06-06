#include "stp/skills/Chip.h"

#include "stp/constants/ControlConstants.h"

namespace rtt::ai::stp::skill {

Status Chip::onUpdate(const StpInfo &info) noexcept {
    float chipVelocity = std::clamp(info.getKickChipVelocity(), stp::control_constants::MIN_CHIP_POWER, stp::control_constants::MAX_CHIP_POWER);
    command.kickType = KickType::CHIP;
    command.kickSpeed = chipVelocity;

    command.dribblerOn = info.getDribblerOn();

    command.yaw = info.getRobot().value()->getYaw();

    if (chipAttempts > control_constants::MAX_CHIP_ATTEMPTS) {
        command.waitForBall = false;
        chipAttempts = 0;
    } else {
        command.waitForBall = true;
    }

    command.id = info.getRobot().value()->getId();

    forwardRobotCommand();

    if (info.getBall()->get()->velocity.length() > stp::control_constants::HAS_CHIPPED_ERROR_MARGIN) {
        chipAttempts = 0;
        return Status::Success;
    }

    ++chipAttempts;
    return Status::Running;
}

const char *Chip::getName() { return "Chip"; }

}  // namespace rtt::ai::stp::skill