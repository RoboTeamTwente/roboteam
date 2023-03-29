//
// Created by jordi on 09-03-20.
//

#include "stp/skills/GoToPos.h"

//#include "control/positionControl/BBTrajectories/WorldObjects.h"
#include "stp/computations/PositionComputations.h"
#include "utilities/GameStateManager.hpp"
#include "world/World.hpp"

#include <os/log.h>
#include <os/signpost.h>


namespace rtt::ai::stp::skill {

os_log_t log_handle = os_log_create("com.jviotti.my-app", OS_LOG_CATEGORY_POINTS_OF_INTEREST);
os_signpost_id_t signpost_id = os_signpost_id_generate(log_handle);
//assert(signpost_id != OS_SIGNPOST_ID_INVALID);

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
    os_signpost_interval_begin(log_handle, signpost_id, "My first interval", "Begin metadata: %s", "Foo");

//    rtt::control::CommandCollision commandCollision;
    const auto positionControlCmd = info.getCurrentWorld()->getRobotPositionController()->computeNextControlCommand({
        .robotId = robot.value()->getId(),
        .state = {robot.value()->getPos(), robot.value()->getVel()},
        .targetPos = targetPos,
        .maxVel = info.getMaxRobotVelocity(),
        .avoidObjects = avoidObj
    }, info.getPidType().value());
    os_signpost_interval_end(log_handle, signpost_id, "My first interval", "End metadata: %s", "Foo");


//    bool useOldPathPlanning = false;
//    rtt::control::CommandCollision commandCollision;
//
//    if (useOldPathPlanning) {
//        // Calculate commands from path planning and tracking
//        commandCollision.robotCommand = info.getCurrentWorld()->getRobotPositionController()->computeAndTrackPath(
//            info.getField().value(), info.getRobot().value()->getId(), info.getRobot().value()->getPos(), info.getRobot().value()->getVel(), targetPos, info.getPidType().value());
//    } else {
//        // _______Use this one for the BBT pathplanning and tracking_______
//        commandCollision = info.getCurrentWorld()->getRobotPositionController()->computeAndTrackTrajectory(
//            info.getCurrentWorld(), info.getField().value(), info.getRobot().value()->getId(), info.getRobot().value()->getPos(), info.getRobot().value()->getVel(), targetPos,
//            info.getMaxRobotVelocity(), info.getPidType().value(), avoidObj);
//    }
//
    if (positionControlCmd.isOccupied) {
        return Status::Failure;
    }

//    double targetVelocityLength;
//    if (info.getPidType() == stp::PIDType::KEEPER && (info.getRobot()->get()->getPos() - targetPos).length() > 2.0 * control_constants::ROBOT_RADIUS) {
//        targetVelocityLength = std::max(std::clamp(positionControlCmd.robotCommand.velocity.length(), 0.0, info.getMaxRobotVelocity()), 1.5); // TODO: Tune this value better
//    } else if (info.getPidType() == stp::PIDType::INTERCEPT && (info.getRobot()->get()->getPos() - targetPos).length() > 2.0 * control_constants::ROBOT_RADIUS) {
//        targetVelocityLength = std::max(std::clamp(positionControlCmd.robotCommand.velocity.length(), 0.0, info.getMaxRobotVelocity()), 1.5); // TODO: Tune this value better
//    } else {
//        targetVelocityLength = std::clamp(positionControlCmd.robotCommand.velocity.length(), 0.0, info.getMaxRobotVelocity());
//    }
//    // Clamp and set velocity
//    Vector2 targetVelocity = positionControlCmd.robotCommand.velocity.stretchToLength(targetVelocityLength);

//     Set velocity and angle commands
    command.velocity = positionControlCmd.robotCommand.velocity;
//    command.velocity = Vector2{1, 1};

    command.targetAngle = positionControlCmd.robotCommand.targetAngle;

    // Clamp and set dribbler speed
    int targetDribblerPercentage = std::clamp(info.getDribblerSpeed(), 0, 100);
    double targetDribblerSpeed = targetDribblerPercentage / 100.0 * stp::control_constants::MAX_DRIBBLER_CMD;

    // Set dribbler speed command
    command.dribblerSpeed = targetDribblerSpeed;

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