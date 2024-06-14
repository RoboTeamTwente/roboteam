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

    // Don't go to the ball if we are close to the ball and the angle is too big
    // If the ball is moving, we move to the interception point with the front assembly, and the rest of the robot is more away from the ball
    // Something like: ball ...... interceptionPoint. robot
    // If the ball is not moving (or slow), we move to the interception point with the center of the robot, and the rest of the robot is more away from the ball
    // This makes sure we always hit the ball with our front assembly, and we never go to the wrong 'side' of the ball.
    if (info.getRobot()->get()->getAngleDiffToBall() > constants::HAS_BALL_ANGLE && distanceToBall < constants::ROBOT_CLOSE_TO_POINT) {
        skillStpInfo.setPositionToMoveTo(info.getRobot()->get()->getPos());
    } else if (info.getBall()->get()->velocity.length() > constants::BALL_IS_MOVING_SLOW_LIMIT) {
        auto newRobotPos = interceptionPosition + (interceptionPosition - ballPosition).stretchToLength(constants::CENTER_TO_FRONT);
        skillStpInfo.setPositionToMoveTo(newRobotPos);
    } else {
        auto getBallDistance = std::max(distanceToInterception - constants::CENTER_TO_FRONT, MIN_DISTANCE_TO_TARGET);
        Vector2 newRobotPosition = robotPosition + (interceptionPosition - robotPosition).stretchToLength(getBallDistance);
        newRobotPosition = FieldComputations::projectPointToValidPosition(info.getField().value(), newRobotPosition, info.getObjectsToAvoid());
        skillStpInfo.setPositionToMoveTo(newRobotPosition);
    }

    skillStpInfo.setYaw((ballPosition - robotPosition).angle());

    if (distanceToBall < constants::TURN_ON_DRIBBLER_DISTANCE) {
        skillStpInfo.setDribblerOn(true);
    }

    return skillStpInfo;
}

bool GetBall::isTacticFailing(const StpInfo &) noexcept { return false; }

bool GetBall::shouldTacticReset(const StpInfo &info) noexcept { return (!info.getRobot()->get()->hasBall() && skills.current_num() == 1); }

bool GetBall::isEndTactic() noexcept { return false; }

const char *GetBall::getName() { return "Get Ball"; }

}  // namespace rtt::ai::stp::tactic
