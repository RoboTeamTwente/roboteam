#include "stp/tactics/active/ChipAtPos.h"

#include "control/ControlUtils.h"
#include "stp/constants/ControlConstants.h"
#include "stp/skills/Chip.h"
#include "stp/skills/OrbitAngular.h"

namespace rtt::ai::stp::tactic {

ChipAtPos::ChipAtPos() { skills = rtt::collections::state_machine<Skill, Status, StpInfo>{skill::OrbitAngular(), skill::Chip()}; }

std::optional<StpInfo> ChipAtPos::calculateInfoForSkill(const StpInfo &info) noexcept {
    if (!info.getPositionToShootAt() || !info.getRobot() || !info.getBall()) return std::nullopt;

    StpInfo skillStpInfo = info;
    auto angleToTarget = (info.getPositionToShootAt().value() - info.getRobot().value()->getPos()).angle();
    skillStpInfo.setYaw(angleToTarget);

    auto distanceBallToTarget = (info.getBall()->get()->position - info.getPositionToShootAt().value()).length();
    skillStpInfo.setKickChipVelocity(control::ControlUtils::determineChipForce(distanceBallToTarget));

    if (skills.current_num() == 0) {
        skillStpInfo.setDribblerOn(true);
    }

    return skillStpInfo;
}

bool ChipAtPos::isEndTactic() noexcept { return false; }

bool ChipAtPos::isTacticFailing(const StpInfo &info) noexcept {
    if (skills.current_num() != 1) {
        return info.getRobot().value()->hasBall() && !info.getPositionToShootAt();
    }
    return false;
}

bool ChipAtPos::shouldTacticReset(const StpInfo &info) noexcept {
    if (skills.current_num() != 0) {
        auto errorMargin = stp::control_constants::GO_TO_POS_ANGLE_ERROR_MARGIN * M_PI;
        return info.getRobot().value()->getYaw().shortestAngleDiff(info.getYaw()) > errorMargin;
    }
    return false;
}

const char *ChipAtPos::getName() { return "Chip At Pos"; }

}  // namespace rtt::ai::stp::tactic
