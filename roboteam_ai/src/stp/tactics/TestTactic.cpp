#include "stp/tactics/TestTactic.h"

#include "stp/skills/TestSkill.h"

namespace rtt::ai::stp {

TestTactic::TestTactic() { skills = rtt::collections::state_machine<Skill, Status, StpInfo>{skill::TestSkill()}; }

std::optional<StpInfo> TestTactic::calculateInfoForSkill(StpInfo const &info) noexcept {
    StpInfo skillStpInfo = info;
    if (!skillStpInfo.getField()) return std::nullopt;
    return skillStpInfo;
}

bool TestTactic::isTacticFailing(const StpInfo &) noexcept { return false; }

bool TestTactic::shouldTacticReset(const StpInfo &) noexcept { return false; }

bool TestTactic::isEndTactic() noexcept { return false; }

const char *TestTactic::getName() { return "Test Tactic"; }

}  // namespace rtt::ai::stp
