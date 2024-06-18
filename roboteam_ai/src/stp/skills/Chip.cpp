#include "stp/skills/Chip.h"

#include "utilities/Constants.h"

namespace rtt::ai::stp::skill {

Status Chip::onUpdate(const StpInfo &info) noexcept {
    float chipVelocity = std::clamp(info.getKickChipVelocity(), constants::MIN_CHIP_POWER, constants::MAX_CHIP_POWER);
    command.kickType = KickType::CHIP;
    command.kickSpeed = chipVelocity;

    command.dribblerOn = info.getDribblerOn();

    command.yaw = info.getRobot().value()->getYaw();

    command.waitForBall = true;

    command.id = info.getRobot().value()->getId();

    forwardRobotCommand();

    if (info.getBall()->get()->velocity.length() > constants::HAS_CHIPPED_ERROR_MARGIN) {
        return Status::Success;
    }

    return Status::Running;
}

const char *Chip::getName() { return "Chip"; }

}  // namespace rtt::ai::stp::skill