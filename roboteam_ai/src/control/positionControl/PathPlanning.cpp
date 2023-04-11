//
// Created by martin on 14-5-22.
//

#ifndef RTT_PATHPLANNING_H
#define RTT_PATHPLANNING_H

#include "control/positionControl/PathPlanning.h"

#include <functional>
#include <ranges>

#include "control/positionControl/BBTrajectories/BBTrajectory2D.h"
#include "control/positionControl/PositionControlUtils.h"
#include "interface/widgets/widget.h"

namespace rtt::ai::control {

PathPlanning::PathPlanning(const CollisionDetector& collisionDetector) : collisionDetector(collisionDetector) {}

void PathPlanning::generateNewPath(std::vector<StateVector>& pathBuffer, const PositionControlInput& input) const noexcept {
    const auto directTrajectory = BBTrajectory2D(input.state.position, input.state.velocity, input.targetPos, input.maxVel, ai::Constants::MAX_ACC_UPPER());

    const auto steps = std::views::iota(0, PositionControlUtils::convertTimeToStep(directTrajectory.getTotalTime()));
    const auto collision = std::find_if(steps.begin(), steps.end(), [&](int step) {
        const auto position = directTrajectory.getPosition(PositionControlUtils::convertStepToTime(step));
        return collisionDetector.doesCollideWithMovingObjects(position, input.robotId, input.avoidObjects, step) ||
               collisionDetector.doesCollideWithStaticObjects(position, input.avoidObjects);
    });

    const double directTrajectoryCost =
        scorePath(directTrajectory.getTotalTime(), 0, collision != steps.end() ? std::make_optional(PositionControlUtils::convertStepToTime(*collision)) : std::nullopt);

    if (collision == steps.end()) {
        fillPathBuffer(pathBuffer, directTrajectory);
        return;
    }

    const auto points = generateIntermediatePoints(input.state.position);

    std::vector<Vector2> myvector{points.begin(), points.end()};
    interface::Input::drawData(interface::Visual::PATHFINDING_DEBUG, myvector, Qt::yellow, input.robotId, interface::Drawing::CROSSES);

    auto trajectories = std::array<ScoredTrajectoryPair, POINTS_COUNT>();
    std::transform(points.begin(), points.end(), trajectories.begin(), [&](const Vector2& point) { return findTrajectoryForPoint(directTrajectory, directTrajectoryCost, input, point); });

    const auto bestTrajectory = std::min_element(trajectories.begin(), trajectories.end(), [](const auto a, const auto b) { return a.cost < b.cost; });
    fillPathBuffer(pathBuffer, *bestTrajectory);

    int limiter = 0;
    std::for_each(trajectories.begin(), trajectories.end(), [&](const auto& trajectory) {
        //        if (limiter > 1) return;

        //        std::vector<StateVector> myvector;
        //        fillPathBuffer(myvector, trajectory);
        //        std::vector<Vector2> drawingBuffer;
        //        std::transform(myvector.begin(), myvector.end(), std::back_inserter(drawingBuffer), [](const auto& state) { return state.position; });
        //        interface::Input::drawData(interface::Visual::PATHFINDING, {Vector2{}, Vector2{1, 1}}, Qt::yellow, input.robotId, interface::Drawing::LINES_CONNECTED);
        //        interface::Input::drawData(interface::Visual::PATHFINDING, drawingBuffer, Qt::yellow, input.robotId, interface::Drawing::LINES_CONNECTED);
        //        limiter++;
    });
}

std::array<Vector2, PathPlanning::POINTS_COUNT> PathPlanning::generateIntermediatePoints(const Vector2& center) const {
    auto intermediatePoints = std::array<Vector2, POINTS_COUNT>();
    int counter = 0;
    for (auto offset : RADIUS_OFFSETS) {
        auto pointToRotate = center + Vector2{0, fieldWidth / offset};
        for (int i = 0; i < POINTS_PER_CIRCLE; i++) {
            intermediatePoints[counter] = (pointToRotate.rotateAroundPoint(i * ANGLE_BETWEEN_POINTS, center));
            counter++;
        }
    }

    return intermediatePoints;
}

ScoredTrajectoryPair PathPlanning::findTrajectoryForPoint(const BBTrajectory2D directTrajectory, const int directTrajectoryCost, const PositionControlInput& input, const Vector2& intermediatePoint) const {
    int lowestCost = directTrajectoryCost;
    std::pair<int, BBTrajectory2D> bestEndTrajectory = {0, directTrajectory};

    std::vector<StateVector> stateVector;
    std::vector<Vector2> drawingBuffer;

    const auto startTrajectory = BBTrajectory2D(input.state.position, input.state.velocity, intermediatePoint, input.maxVel, ai::Constants::MAX_ACC_UPPER());
    fillPathBuffer(stateVector, startTrajectory);
    std::transform(stateVector.begin(), stateVector.end(), std::back_inserter(drawingBuffer), [](const auto& state) { return state.position; });
    interface::Input::drawData(interface::Visual::PATHFINDING_DEBUG, drawingBuffer, Qt::blue, input.robotId, interface::Drawing::LINES_CONNECTED);
    stateVector.clear();
    drawingBuffer.clear();

    for (int step = 0; PositionControlUtils::convertStepToTime(step) <= startTrajectory.getTotalTime(); step += 2) {
        const double startTrajectoryTime = PositionControlUtils::convertStepToTime(step);
        const auto state = StateVector{startTrajectory.getPosition(startTrajectoryTime), startTrajectory.getVelocity(startTrajectoryTime)};
        if (collisionDetector.doesCollideWithMovingObjects(state.position, input.robotId, input.avoidObjects, step)) {
            break;
        }

        const auto& [endTrajectory, collision] = trajectoryFromState(step, state, input);
        const int pathCost = scorePath(startTrajectoryTime, endTrajectory.getTotalTime(), collision);

        if (pathCost < lowestCost) {
            lowestCost = pathCost;
            bestEndTrajectory = {step, endTrajectory};
        }

        if (!collision.has_value()) {
            break;
        }
    }

    ScoredTrajectoryPair scoredTrajectoryPair = {
        .start = startTrajectory,
        .end = bestEndTrajectory.second,
        .cutoffIndex = bestEndTrajectory.first,
        .cost = lowestCost,
    };

    fillPathBuffer(stateVector, scoredTrajectoryPair.end);
    std::transform(stateVector.begin(), stateVector.end(), std::back_inserter(drawingBuffer), [](const auto& state) { return state.position; });
    //    interface::Input::drawData(interface::Visual::PATHFINDING, drawingBuffer, Qt::green, input.robotId, interface::Drawing::CROSSES);
    stateVector.clear();
    drawingBuffer.clear();

    return scoredTrajectoryPair;
}

std::pair<BBTrajectory2D, std::optional<double>> PathPlanning::trajectoryFromState(int stepOffset, const StateVector& forState, const PositionControlInput& forInput) const {
    const auto trajectory = BBTrajectory2D(forState.position, forState.velocity, forInput.targetPos, forInput.maxVel, ai::Constants::MAX_ACC_UPPER());

    const auto steps = std::views::iota(0, PositionControlUtils::convertTimeToStep(trajectory.getTotalTime()));
    std::for_each(steps.begin(), steps.end(), [&](int step) {
        const double time = PositionControlUtils::convertStepToTime(step);
        const auto position = trajectory.getPosition(time);
        interface::Input::drawData(interface::Visual::PATHFINDING_DEBUG, {position}, Qt::cyan, forInput.robotId, interface::Drawing::CROSSES);
    });

    auto collision = std::find_if(steps.begin(), steps.end(), [&](int step) {
        const double time = PositionControlUtils::convertStepToTime(step);
        const auto position = trajectory.getPosition(time);
        interface::Input::drawData(interface::Visual::PATHFINDING_DEBUG, {position}, Qt::magenta, forInput.robotId, interface::Drawing::CROSSES);
        return collisionDetector.doesCollideWithMovingObjects(position, forInput.robotId, forInput.avoidObjects, stepOffset + step) ||
               collisionDetector.doesCollideWithStaticObjects(position, forInput.avoidObjects);
    });

    return {trajectory, collision != steps.end() ? std::make_optional(PositionControlUtils::convertStepToTime(*collision)) : std::nullopt};
}

int PathPlanning::scorePath(double startTrajectoryDuration, double endTrajectoryDuration, std::optional<double> collisionTime) {
    //    auto cost = PositionControlUtils::convertTimeToStep(startTrajectoryDuration + endTrajectoryDuration);
    int timeStepsCount = PositionControlUtils::convertTimeToStep(startTrajectoryDuration + endTrajectoryDuration);
    int cost = static_cast<int>(std::round(timeStepsCount * log(timeStepsCount)));
    if (collisionTime.has_value()) {
        cost += 1500;  // Fixed penalty for collision
        cost -= static_cast<int>(std::round(startTrajectoryDuration + collisionTime.value()));
    }

    return cost;
}

void PathPlanning::fillPathBuffer(std::vector<StateVector>& pathBuffer, const ScoredTrajectoryPair& trajectoryPair) {
    auto steps = std::views::iota(0, trajectoryPair.cutoffIndex);
    for (int step : steps) {
        const auto time = PositionControlUtils::convertStepToTime(step);
        pathBuffer.emplace_back(StateVector{trajectoryPair.start.getPosition(time), trajectoryPair.start.getVelocity(time)});
    }

    steps = std::views::iota(0, PositionControlUtils::convertTimeToStep(trajectoryPair.end.getTotalTime()));
    for (int step : steps) {
        const auto time = PositionControlUtils::convertStepToTime(step);
        pathBuffer.emplace_back(StateVector{trajectoryPair.end.getPosition(time), trajectoryPair.end.getVelocity(time)});
    }
}

void PathPlanning::fillPathBuffer(std::vector<StateVector>& pathBuffer, const BBTrajectory2D& trajectory) {
    const auto steps = std::views::iota(0, PositionControlUtils::convertTimeToStep(trajectory.getTotalTime()));
    for (int step : steps) {
        const auto time = PositionControlUtils::convertStepToTime(step);
        pathBuffer.emplace_back(StateVector{trajectory.getPosition(time), trajectory.getVelocity(time)});
    }
}

void PathPlanning::updateConstraints(const rtt::Field& field) { fieldWidth = field.playArea.width(); }

}  // namespace rtt::ai::control
#endif  // RTT_PATHPLANNING_H
