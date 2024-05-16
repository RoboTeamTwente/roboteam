#include "stp/tactics/passive/Halt.h"

#include "stp/skills/Rotate.h"

namespace rtt::ai::stp::tactic {

Halt::Halt() { skills = collections::state_machine<Skill, Status, StpInfo>{skill::Rotate()}; }

std::optional<StpInfo> Halt::calculateInfoForSkill(StpInfo const &info) noexcept {
    StpInfo skillInfo = info;
    skillInfo.setAngle(0.0);

    return skillInfo;
}

bool Halt::isTacticFailing(const StpInfo &) noexcept { return false; }

bool Halt::shouldTacticReset(const StpInfo &) noexcept { return false; }

bool Halt::isEndTactic() noexcept { return true; }

const char *Halt::getName() { return "Halt"; }

}  // namespace rtt::ai::stp::tactic
