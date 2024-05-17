#include "stp/tactics/passive/Formation.h"

#include "stp/skills/GoToPos.h"

// This tactic is used to make robots move to a certain position, without any other actions

namespace rtt::ai::stp::tactic {

Formation::Formation() { skills = rtt::collections::state_machine<Skill, Status, StpInfo>{skill::GoToPos()}; }

std::optional<StpInfo> Formation::calculateInfoForSkill(StpInfo const &info) noexcept {
    StpInfo skillStpInfo = info;

    if (!info.getPositionToMoveTo()) return std::nullopt;
    if (!info.getAngle()) skillStpInfo.setAngle(0);
    skillStpInfo.setDribblerSpeed(0);

    return skillStpInfo;
}

bool Formation::isTacticFailing(const StpInfo &info) noexcept { return !info.getPositionToMoveTo(); }

bool Formation::shouldTacticReset(const StpInfo &) noexcept { return false; }

bool Formation::isEndTactic() noexcept { return true; }

const char *Formation::getName() { return "Formation"; }

}  // namespace rtt::ai::stp::tactic