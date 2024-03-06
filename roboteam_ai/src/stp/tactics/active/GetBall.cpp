//
// Created by ratoone on 10-03-20.
/// Moves the robot to the BALL in a straight line and ROTATES the robot towards to ball.

/// ACTIVE
//

#include "stp/tactics/active/GetBall.h"

#include <world/World.hpp>

#include "control/ControlUtils.h"
#include "stp/skills/GoToPos.h"
#include "stp/skills/Rotate.h"
#include "utilities/GameStateManager.hpp"
#include "world/FieldComputations.h"

namespace rtt::ai::stp::tactic {
GetBall::GetBall() { skills = collections::state_machine<Skill, Status, StpInfo>{skill::GoToPos()}; }

std::optional<StpInfo> GetBall::calculateInfoForSkill(StpInfo const &info) noexcept {
    StpInfo skillStpInfo = info;

    if (!skillStpInfo.getRobot() || !skillStpInfo.getBall() || !skillStpInfo.getField()) return std::nullopt;

    Vector2 robotPosition = skillStpInfo.getRobot().value()->getPos();
    Vector2 robotVelocity = skillStpInfo.getRobot().value()->getVel();
    Vector2 interceptionPosition;
    Vector2 ballPosition = skillStpInfo.getBall().value()->position;
    auto maxRobotVelocity = GameStateManager::getCurrentGameState().getRuleSet().getMaxRobotVel();
    if (skillStpInfo.getRobot()->get()->hasBall()) {
        maxRobotVelocity = std::clamp(skillStpInfo.getBall().value()->velocity.length() * 0.8, 0.5, maxRobotVelocity);
        skillStpInfo.setMaxRobotVelocity(maxRobotVelocity);
    }
    for (double loopTime = 0; loopTime < 1; loopTime += 0.1) {
        interceptionPosition = FieldComputations::getBallPositionAtTime(*(skillStpInfo.getCurrentWorld()->getWorld()->getBall()->get()), loopTime);
        if (skillStpInfo.getObjectsToAvoid().shouldAvoidOutOfField && !skillStpInfo.getField().value().playArea.contains(interceptionPosition, control_constants::BALL_RADIUS)) {
            break;
        }
        if (skillStpInfo.getObjectsToAvoid().shouldAvoidDefenseArea) {
            if (skillStpInfo.getField().value().leftDefenseArea.contains(interceptionPosition)) {
                std::vector<rtt::Vector2> intersections = FieldComputations::getDefenseArea(skillStpInfo.getField().value(), true, 0, 0)
                                                              .intersections({interceptionPosition, skillStpInfo.getBall().value()->expectedEndPosition});
                if (intersections.size() == 1) interceptionPosition = intersections.at(0);
            } else if (skillStpInfo.getField().value().rightDefenseArea.contains(interceptionPosition)) {
                std::vector<rtt::Vector2> intersections = FieldComputations::getDefenseArea(skillStpInfo.getField().value(), false, 0, 0)
                                                              .intersections({interceptionPosition, skillStpInfo.getBall().value()->expectedEndPosition});
                if (intersections.size() == 1) interceptionPosition = intersections.at(0);
            }
        }

        auto trajectory = Trajectory2D(robotPosition, robotVelocity, interceptionPosition, maxRobotVelocity, ai::Constants::MAX_ACC_UPPER());
        if (trajectory.getTotalTime() < loopTime) {
            break;
        }
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
