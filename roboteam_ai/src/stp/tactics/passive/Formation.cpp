#include "stp/tactics/passive/Formation.h"

#include "stp/skills/GoToPos.h"

namespace rtt::ai::stp::tactic {

Formation::Formation() {
    // Create state machine of skills and initialize first skill
    skills = rtt::collections::state_machine<Skill, Status, StpInfo>{skill::GoToPos()};
}

std::optional<StpInfo> Formation::calculateInfoForSkill(StpInfo const &info) noexcept {
    StpInfo skillStpInfo = info;

    if (!info.getPositionToMoveTo()) return std::nullopt;
    if (!info.getAngle()) skillStpInfo.setAngle(0);

    // Be 100% sure the dribbler is off during the formation
    skillStpInfo.setDribblerSpeed(0);

    return skillStpInfo;
}

bool Formation::isTacticFailing(const StpInfo &info) noexcept {
    // Formation tactic fails if there is no location to move to
    return !info.getPositionToMoveTo();
}

bool Formation::shouldTacticReset(const StpInfo &) noexcept { return false; }

bool Formation::isEndTactic() noexcept {
    // Formation tactic is an end tactic
    return true;
}

const char *Formation::getName() { return "Formation"; }

}  // namespace rtt::ai::stp::tactic