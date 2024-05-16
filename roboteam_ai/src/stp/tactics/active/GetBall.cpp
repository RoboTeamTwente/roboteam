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

namespace rtt::ai::stp::tactic {
GetBall::GetBall() { skills = collections::state_machine<Skill, Status, StpInfo>{skill::GoToPos()}; }

std::optional<StpInfo> GetBall::calculateInfoForSkill(StpInfo const &info) noexcept {
    StpInfo skillStpInfo = info;

    if (!skillStpInfo.getRobot() || !skillStpInfo.getBall() || !skillStpInfo.getField() || !skillStpInfo.getCurrentWorld()) return std::nullopt;
    skillStpInfo.setShouldAvoidBall(false);
    Vector2 robotPosition = skillStpInfo.getRobot().value()->getPos();
    Vector2 ballPosition = skillStpInfo.getBall().value()->position;
    auto maxRobotVelocity = GameStateManager::getCurrentGameState().getRuleSet().getMaxRobotVel();
    InterceptionInfo interceptionInfo =
        InterceptionComputations::calculateInterceptionInfo(std::vector<world::view::RobotView>(1, skillStpInfo.getRobot().value()), skillStpInfo.getCurrentWorld());
    Vector2 interceptionPosition = interceptionInfo.interceptLocation;
    if (skillStpInfo.getRobot()->get()->hasBall()) {
        maxRobotVelocity = std::clamp(skillStpInfo.getBall().value()->velocity.length() * 0.8, 0.5, maxRobotVelocity);
        skillStpInfo.setMaxRobotVelocity(maxRobotVelocity);
    }
    // distance to the ball at the time we intercept it
    double distanceToInterception = (interceptionPosition - robotPosition).length() - control_constants::BALL_RADIUS - control_constants::ROBOT_RADIUS +
                                    control_constants::GO_TO_POS_ERROR_MARGIN + 2 * control_constants::BALL_RADIUS;
    // distance to the ball right now
    double distanceToBall = (ballPosition - robotPosition).length() - control_constants::BALL_RADIUS - control_constants::ROBOT_RADIUS + control_constants::GO_TO_POS_ERROR_MARGIN +
                            2 * control_constants::BALL_RADIUS;
    if (skillStpInfo.getRobot()->get()->getAngleDiffToBall() > Constants::HAS_BALL_ANGLE() && distanceToBall < control_constants::ROBOT_CLOSE_TO_POINT) {
        // don't move too close to the ball until the angle to the ball is (roughly) correct
        skillStpInfo.setPositionToMoveTo(
            FieldComputations::projectPointToValidPosition(info.getField().value(), skillStpInfo.getRobot()->get()->getPos(), info.getObjectsToAvoid()));
    } else {
        // We want to keep going towards the ball slowly if we are already close, to make sure we get it
        auto getBallDistance = std::max(distanceToInterception, control_constants::ROBOT_RADIUS);
        Vector2 newRobotPosition = robotPosition + (interceptionPosition - robotPosition).stretchToLength(getBallDistance);
        newRobotPosition = FieldComputations::projectPointToValidPosition(info.getField().value(), newRobotPosition, info.getObjectsToAvoid());
        skillStpInfo.setPositionToMoveTo(newRobotPosition);
    }
    skillStpInfo.setAngle((ballPosition - robotPosition).angle());

    if (distanceToBall < control_constants::TURN_ON_DRIBBLER_DISTANCE) {
        skillStpInfo.setDribblerSpeed(100);
    }

    return skillStpInfo;
}

bool GetBall::isTacticFailing(const StpInfo &) noexcept { return false; }

bool GetBall::shouldTacticReset(const StpInfo &info) noexcept { return (!info.getRobot()->get()->hasBall() && skills.current_num() == 1); }

bool GetBall::isEndTactic() noexcept {
    // This is not an end tactic
    return false;
}

const char *GetBall::getName() { return "Get Ball"; }

}  // namespace rtt::ai::stp::tactic
