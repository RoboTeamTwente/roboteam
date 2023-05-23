//
// Created by jordi on 09-03-20.
//

#include "stp/skills/GoToPos.h"

#include "stp/computations/PositionComputations.h"
#include "utilities/GameStateManager.hpp"
#include "world/World.hpp"

namespace rtt::ai::stp::skill {

Status GoToPos::onUpdate(const StpInfo &info) noexcept {
    Vector2 targetPos = info.getPositionToMoveTo().value();
    if (!FieldComputations::pointIsValidPosition(info.getField().value(), targetPos, info.getObjectsToAvoid())) {
        RTT_WARNING("Target point is not a valid position for robot id: ", info.getRobot().value()->getId())
        targetPos = FieldComputations::projectPointToValidPosition(info.getField().value(), targetPos, info.getObjectsToAvoid());
    }

    auto avoidObj = info.getObjectsToAvoid();

    if (GameStateManager::getCurrentGameState().getRuleSet().title == "stop") {
        targetPos = PositionComputations::calculateAvoidBallPosition(targetPos, info.getBall()->get()->position, info.getField().value());
        avoidObj.avoidBallDist = control_constants::AVOID_BALL_DISTANCE;
        avoidObj.shouldAvoidBall = true;
    }

    const auto nextPosition = info.getCurrentWorld()->getRobotPositionController()->computeNextPosition({
        .robotId = robot.value()->getId(),
        .state = {robot.value()->getPos(), robot.value()->getVel()},
        .targetPos = targetPos,
        .maxVel = info.getMaxRobotVelocity(),
        .avoidObjects = avoidObj
    }, info.getPidType().value());


    if (nextPosition.has_value()) {
        command.velocity = {nextPosition->x, nextPosition->y};
        command.targetAngle = nextPosition->rot;

//        TODO: What is the purpose of this? Shouldn't the path planing already received max possible velocity?
//              I mean, bang-bang generates optimal as well, changing it will mess up with the path won't it?
//        double targetVelocityLength = 1.0;


//            targetVelocityLength = std::max(std::clamp(command.velocity.length(), 0.0, info.getMaxRobotVelocity()), 1.5);  // TODO: Tune this value better
//        } else if (info.getPidType() == stp::PIDType::INTERCEPT && (info.getRobot()->get()->getPos() - targetPos).length() > 2.0 * control_constants::ROBOT_RADIUS) {
//            targetVelocityLength = std::max(std::clamp(command.velocity.length(), 0.0, info.getMaxRobotVelocity()), 1.5);  // TODO: Tune this value better
//        } else {
//            targetVelocityLength = std::clamp(command.velocity.length(), 0.0, info.getMaxRobotVelocity());
//        }
        // Clamp and set velocity

        double targetVelocityLength = std::clamp(command.velocity.length(), 0.0, info.getMaxRobotVelocity());
        command.velocity = command.velocity.stretchToLength(targetVelocityLength);

        // Clamp and set dribbler speed
        int targetDribblerPercentage = std::clamp(info.getDribblerSpeed(), 0, 100);
        double targetDribblerSpeed = targetDribblerPercentage / 100.0 * stp::control_constants::MAX_DRIBBLER_CMD;

        // Set dribbler speed command
        command.dribblerSpeed = targetDribblerSpeed;
    }

    // set command ID
    command.id = info.getRobot().value()->getId();

    // forward the generated command to the ControlModule, for checking and limiting
    forwardRobotCommand(info.getCurrentWorld());

    // Check if successful
    if ((info.getRobot().value()->getPos() - targetPos).length() <= stp::control_constants::GO_TO_POS_ERROR_MARGIN) {
        return Status::Success;
    } else {
        return Status::Running;
    }
}

const char *GoToPos::getName() { return "Go To Position"; }

}  // namespace rtt::ai::stp::skill