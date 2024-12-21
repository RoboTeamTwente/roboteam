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

    // Store initial ball position when tactic first starts
    if (!initialized) {
        initialBallPos = info.getBall().value()->position;
        initialized = true;
    }

    auto currentPos = info.getRobot().value()->getPos();
    auto world = info.getCurrentWorld();
    
    if (world) {
        Vector2 newPos = PositionComputations::calculatePasserPosition(currentPos, world, info.getRobot().value()->getId(), initialBallPos);
        double angleToTarget = (info.getPositionToShootAt().value() - info.getRobot().value()->getPos()).angle();
        
        skillStpInfo.setPositionToMoveTo(newPos);
        skillStpInfo.setYaw(angleToTarget);
    }

    return skillStpInfo;
}

bool SmoothPass::isTacticFailing(const StpInfo &info) noexcept { 
    return !info.getPositionToShootAt() || 
           !info.getBall() || 
           !info.getRobot().value()->hasBall();  // Add this check
}

bool SmoothPass::shouldTacticReset(const StpInfo &info) noexcept {
    if (skills.current_num() != 0) return false;  // Only check during OrbitAndDrive
    
    // Get current robot position and target position
    auto robot = info.getRobot().value();
    auto currentPos = robot->getPos();
    auto targetPos = info.getPositionToMoveTo().value();
    
    // Check angle deviation
    const auto angleErrorMargin = constants::GO_TO_POS_ANGLE_ERROR_MARGIN * M_PI;
    bool angleDeviated = robot->getYaw().shortestAngleDiff(info.getYaw()) > angleErrorMargin;
    
    // Check position deviation
    bool positionDeviated = (currentPos - targetPos).length() > constants::GO_TO_POS_ERROR_MARGIN;
    
    return angleDeviated || positionDeviated;
}

bool SmoothPass::isEndTactic() noexcept {
    return true;
}

const char *SmoothPass::getName() {
    return "Smooth Pass";
}

}  // namespace rtt::ai::stp::tactic