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

    auto interceptionInfo = InterceptionComputations::calculateInterceptionInfo({info.getRobot().value()}, info.getCurrentWorld());
    Vector2 interceptionPosition = interceptionInfo.interceptLocation;
    Vector2 interceptionVelocity = interceptionInfo.interceptVelocity;

    double distanceToInterception = (interceptionPosition - robotPosition).length();
    double distanceToBall = (ballPosition - robotPosition).length();

    // Don't go to the ball if we are close to the ball and the angle is too big
    // If the ball is moving, we move to the interception point with the front assembly, and the rest of the robot is more away from the ball
    // Something like: ball ...... interceptionPoint. robot
    // If the ball is not moving (or slow), we move to the interception point with the center of the robot, and the rest of the robot is more away from the ball
    // This makes sure we always hit the ball with our front assembly, and we never go to the wrong 'side' of the ball.
    if (info.getRobot()->get()->getAngleDiffToBall() > constants::HAS_BALL_ANGLE && distanceToBall < constants::ROBOT_CLOSE_TO_POINT) {
        skillStpInfo.setPositionToMoveTo(info.getRobot()->get()->getPos());
        skillStpInfo.setTargetVelocity(Vector2(0, 0));
    } else {
        auto newRobotPos = interceptionPosition + (interceptionPosition - ballPosition).stretchToLength(constants::CENTER_TO_FRONT);
        skillStpInfo.setPositionToMoveTo(newRobotPos);
        skillStpInfo.setTargetVelocity(interceptionVelocity);
    }
    skillStpInfo.setMaxJerk(constants::MAX_JERK_DEFAULT);

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
