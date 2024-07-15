#include "stp/skills/GoToPos.h"

#include "stp/computations/PositionComputations.h"
#include "world/World.hpp"

namespace rtt::ai::stp::skill {

Status GoToPos::onUpdate(const StpInfo &info) noexcept {
    auto robot = info.getRobot().value();
    auto field = info.getField().value();
    auto avoidObj = info.getObjectsToAvoid();
    auto targetPos = info.getPositionToMoveTo().value();
    auto targetVel = info.getTargetVelocity();
    auto roleName = info.getRoleName();
    auto ballLocation = info.getBall()->get()->position;
    RefCommand currentGameState = GameStateManager::getCurrentGameState().getCommandId();

    if (!FieldComputations::pointIsValidPosition(field, targetPos, avoidObj) && roleName != "ball_placer") {
        targetPos = FieldComputations::projectPointToValidPosition(field, targetPos, avoidObj);
    }

    if (avoidObj.shouldAvoidOurRobots || avoidObj.shouldAvoidTheirRobots) {
        targetPos = PositionComputations::calculateAvoidRobotsPosition(targetPos, info.getCurrentWorld(), robot->getId(), avoidObj, field);
    }

    if (roleName != "ball_placer" && (avoidObj.shouldAvoidBall || currentGameState == RefCommand::BALL_PLACEMENT_US || currentGameState == RefCommand::BALL_PLACEMENT_THEM ||
                                      currentGameState == RefCommand::BALL_PLACEMENT_US_DIRECT)) {
        targetPos = PositionComputations::calculateAvoidBallPosition(targetPos, ballLocation, field, avoidObj);
    }
    auto [vel, acc] = info.getCurrentWorld()->getRobotPositionController()->computeAndTrackTrajectory(
        info.getCurrentWorld(), field, robot->getId(), robot->getPos(), robot->getVel(), targetPos, targetVel, info.getMaxRobotVelocity(), info.getMaxJerk(), avoidObj);
    command.velocity = vel;
    command.acceleration = acc;

    command.yaw = info.getYaw();

    command.dribblerOn = info.getDribblerOn();

    // set command ID
    command.id = robot->getId();

    if (info.getShootOnFirstTouch() && info.getKickOrChip() != rtt::KickType::NO_KICK && info.getPositionToShootAt()) {
        double distanceBallToTarget = (info.getBall()->get()->position - info.getPositionToShootAt().value()).length();
        auto kickChipVelocity = control::ControlUtils::determineKickForce(distanceBallToTarget, info.getShotPower());
        command.kickType = info.getKickOrChip();
        command.kickSpeed = std::clamp(kickChipVelocity, constants::MIN_KICK_POWER, constants::MAX_KICK_POWER);
    }

    // forward the generated command to the ControlModule, for checking and limiting
    forwardRobotCommand();

    // Check if successful
    auto distanceError = (robot->getPos() - targetPos).length();
    if ((robot->hasBall() && info.getRoleName() != "ball_placer") || (robot->hasBall() && distanceError <= constants::BALL_PLACER_MARGIN)) {
        return Status::Success;
    } else {
        return Status::Running;
    }
}

const char *GoToPos::getName() { return "Go To Position"; }

}  // namespace rtt::ai::stp::skill