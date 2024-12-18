#include "stp/tactics/active/SmoothPass.h"
#include "stp/skills/OrbitAngularAndDrive.h"
#include "stp/skills/Kick.h"
#include "stp/computations/PositionComputations.h"

namespace rtt::ai::stp::tactic {

SmoothPass::SmoothPass() {
    // Use both orbit-and-drive and kick skills
    skills = rtt::collections::state_machine<Skill, Status, StpInfo>{
        skill::OrbitAngularAndDrive(),
        skill::Kick()
    };
}

std::optional<StpInfo> SmoothPass::calculateInfoForSkill(const StpInfo &info) noexcept {
    if (!info.getPositionToShootAt() || !info.getBall() || !info.getRobot()) return std::nullopt;

    StpInfo skillStpInfo = info;
    
    // If we're in second state (kicking), no additional calculation needed
    if (skills.current_num() == 1) {
        return skillStpInfo;
    }

    auto currentPos = info.getRobot().value()->getPos();
    auto world = info.getCurrentWorld();
    
    if (world) {
        // Use our new computation functions
        Vector2 newPos = PositionComputations::calculatePasserPosition(currentPos, world, info.getRobot().value()->getId());
        Angle newAngle = PositionComputations::calculatePasserAngle(currentPos, world);
        
        skillStpInfo.setPositionToMoveTo(newPos);
        skillStpInfo.setYaw(newAngle);
    }

    return skillStpInfo;
}

bool SmoothPass::isTacticFailing(const StpInfo &info) noexcept { 
    return !info.getPositionToShootAt() || !info.getBall(); 
}

bool SmoothPass::shouldTacticReset(const StpInfo &info) noexcept {
    // Reset if we've deviated too much from desired angle during orbit
    if (skills.current_num() == 0) {  // During OrbitAndDrive
        const auto errorMargin = constants::GO_TO_POS_ANGLE_ERROR_MARGIN * M_PI;
        return info.getRobot().value()->getYaw().shortestAngleDiff(info.getYaw()) > errorMargin;
    }
    return false;
}

bool SmoothPass::isEndTactic() noexcept {
    return true;
}

const char *SmoothPass::getName() {
    return "Smooth Pass";
}

}  // namespace rtt::ai::stp::tactic