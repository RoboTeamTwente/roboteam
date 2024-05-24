#include "stp/tactics/active/GetBall.h"

#include <stp/computations/InterceptionComputations.h>

#include <world/World.hpp>

#include "control/ControlUtils.h"
#include "stp/computations/PositionComputations.h"
#include "stp/computations/PositionScoring.h"
#include "stp/skills/GoToPos.h"
#include "stp/skills/Rotate.h"
#include "utilities/GameStateManager.hpp"
#include "world/FieldComputations.h"

// The minimum distance we set the target position away from the robot
const double MIN_DISTANCE_TO_TARGET = 0.03;

namespace rtt::ai::stp::tactic {
GetBall::GetBall() { skills = collections::state_machine<Skill, Status, StpInfo>{skill::GoToPos()}; }

std::optional<StpInfo> GetBall::calculateInfoForSkill(const StpInfo &info) noexcept {
    if (!info.getRobot() || !info.getBall() || !info.getField() || !info.getCurrentWorld()) return std::nullopt;

    StpInfo skillStpInfo = info;
    skillStpInfo.setShouldAvoidBall(false);

    Vector2 robotPosition = info.getRobot().value()->getPos();
    Vector2 ballPosition = info.getBall().value()->position;

    auto maxRobotVelocity = GameStateManager::getCurrentGameState().getRuleSet().getMaxRobotVel();
    auto interceptionInfo = InterceptionComputations::calculateInterceptionInfo({info.getRobot().value()}, info.getCurrentWorld());
    Vector2 interceptionPosition = interceptionInfo.interceptLocation;

    if (info.getRobot()->get()->hasBall()) {
        maxRobotVelocity = std::clamp(info.getBall().value()->velocity.length() * 0.8, 0.5, maxRobotVelocity);
        skillStpInfo.setMaxRobotVelocity(maxRobotVelocity);
    }

    double distanceToInterception = (interceptionPosition - robotPosition).length();
    double distanceToBall = (ballPosition - robotPosition).length();

    if (info.getRobot()->get()->getYawDiffToBall() > Constants::HAS_BALL_ANGLE() && distanceToBall < control_constants::ROBOT_CLOSE_TO_POINT) {
        skillStpInfo.setPositionToMoveTo(FieldComputations::projectPointToValidPosition(info.getField().value(), info.getRobot()->get()->getPos(), info.getObjectsToAvoid()));
    } else {
        auto getBallDistance = std::max(distanceToInterception - control_constants::CENTER_TO_FRONT, MIN_DISTANCE_TO_TARGET);
        Vector2 newRobotPosition = robotPosition + (interceptionPosition - robotPosition).stretchToLength(getBallDistance);
        newRobotPosition = FieldComputations::projectPointToValidPosition(info.getField().value(), newRobotPosition, info.getObjectsToAvoid());
        skillStpInfo.setPositionToMoveTo(newRobotPosition);
    }

    skillStpInfo.setYaw((ballPosition - robotPosition).angle());

    if (distanceToBall < control_constants::TURN_ON_DRIBBLER_DISTANCE) {
        skillStpInfo.setDribblerOn(true);
    }

    return skillStpInfo;
}

bool GetBall::isTacticFailing(const StpInfo &) noexcept { return false; }

bool GetBall::shouldTacticReset(const StpInfo &info) noexcept { return (!info.getRobot()->get()->hasBall() && skills.current_num() == 1); }

bool GetBall::isEndTactic() noexcept { return false; }

const char *GetBall::getName() { return "Get Ball"; }

}  // namespace rtt::ai::stp::tactic
