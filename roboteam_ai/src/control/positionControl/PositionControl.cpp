//
// Created by ratoone on 18-11-19.
//

#include "control/positionControl/PositionControl.h"

#include <span>

#include "control/positionControl/BBTrajectories/BBTrajectory2D.h"

namespace rtt::ai::control {

PositionControlCommand PositionControl::computeNextControlCommand(const PositionControlInput& input, stp::PIDType pidType) {
    auto command = PositionControlCommand{};
    if (collisionDetector.doesCollideWithMovingObjects(input.targetPos, input.robotId, input.avoidObjects.shouldAvoidBall)) {
        command.isOccupied = true;
        return command;
    }

    auto& path = paths.at(input.robotId);
    if (pathTracking.shouldUpdatePath(input, path.remaining) != DONT_UPDATE) {
        path.full.clear();
        pathPlanning.generateNewPath(path.full, input);
        path.remaining = std::span(path.full);
    }

    const auto [remainingPath, trackingVelocity] = pathTracking.trackPath(input.state, path.remaining, pidControllers.at(input.robotId), pidType);
    path.remaining = remainingPath;
    collisionDetector.updateTimelineForOurRobot(path.remaining, input.state.position, input.robotId);

    drawingBuffer.clear();
    std::transform(remainingPath.begin(), remainingPath.end(), std::back_inserter(drawingBuffer), [](const auto& state) { return state.position; });
    interface::Input::drawData(interface::Visual::PATHFINDING, drawingBuffer, Qt::yellow, input.robotId, interface::Drawing::LINES_CONNECTED);
    interface::Input::drawData(interface::Visual::PATHFINDING, drawingBuffer, Qt::green, input.robotId, interface::Drawing::DOTS);

    collisionDetector.updateTimelineForOurRobot(path.remaining, input.state.position, input.robotId);
    command.robotCommand.velocity = {trackingVelocity.x, trackingVelocity.y};
    command.robotCommand.targetAngle = trackingVelocity.rot;
    return command;
}

void PositionControl::updatePositionControl(std::optional<WorldDataView> world, std::optional<FieldView> field, const GameStateView& gameState) {
    if (!world.has_value() || !field.has_value()) { return; }

    collisionDetector.updateDefenseAreas(field);
    collisionDetector.setMinBallDistance(gameState.getRuleSet().minDistanceToBall);
    collisionDetector.updateTimeline(world->getRobotsNonOwning(), world->getBall());

    pathPlanning.updateConstraints(field.value());
    for (const auto& ourRobot : world->getUs()) {
        if (paths.contains(ourRobot->getId())) { continue; }

        auto emptyPath = std::vector<StateVector>();
        emptyPath.reserve(512);
        paths.insert({ourRobot->getId(), {.full = emptyPath, .remaining = std::span(emptyPath)}});

        auto xPID = PID();
        auto yPID = PID();
        xPID.setMaxIOutput(Constants::MAX_VEL());
        yPID.setMaxIOutput(Constants::MAX_VEL());

        pidControllers.insert({ourRobot->getId(), {xPID, yPID}});
    }
}

PositionControl::PositionControl(): pathTracking(PathTracking(collisionDetector)), pathPlanning(PathPlanning(collisionDetector)) {
    drawingBuffer.reserve(512);
}
}  // namespace rtt::ai::control