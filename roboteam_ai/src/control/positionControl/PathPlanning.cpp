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

void PathPlanning::generateNewPath(std::vector<StateVector>& pathBuffer, const PositionControlInput& input) const noexcept  {
    const auto trajectory = BBTrajectory2D(input.state.position, input.state.velocity, input.targetPos, input.maxVel, ai::Constants::MAX_ACC_UPPER());

    const auto steps = std::views::iota(0, PositionControlUtils::convertTimeToStep(trajectory.getTotalTime()));
    bool doesCollide = std::any_of(steps.begin(), steps.end(), [&](int step) {
        const auto position = trajectory.getPosition(PositionControlUtils::convertStepToTime(step));
        return collisionDetector.doesCollideWithMovingObjects(position, input.robotId, input.avoidObjects, step) ||
               collisionDetector.doesCollideWithStaticObjects(position, input.avoidObjects);
    });

    if (!doesCollide) {
        fillPathBuffer(pathBuffer, trajectory);
        return;
    }

    const auto points = generateIntermediatePoints(input.state.position);
    auto trajectories = std::array<ScoredTrajectoryPair, POINTS_COUNT>();
    std::transform(points.begin(), points.end(), trajectories.begin(), [&](const Vector2& point) { return findTrajectoryForPoint(input, point); });

    const auto bestTrajectory = std::min_element(trajectories.begin(), trajectories.end(), [](const auto a, const auto b) { return a.cost < b.cost; });
    fillPathBuffer(pathBuffer, *bestTrajectory);
}

std::array<Vector2, PathPlanning::POINTS_COUNT> PathPlanning::generateIntermediatePoints(const Vector2& center) const {
    auto intermediatePoints = std::array<Vector2, POINTS_COUNT>();
    int counter = 0;
    for (auto offset : RADIUS_OFFSETS) {
        auto pointToRotate = center + Vector2{0, fieldWidth / offset};
        for (int i = 0; i < POINTS_PER_CIRCLE; i++) {
            intermediatePoints[counter] = (pointToRotate.rotateAroundPoint(i * POINTS_PER_CIRCLE, center));
            counter++;
        }
    }

    return intermediatePoints;
}

ScoredTrajectoryPair PathPlanning::findTrajectoryForPoint(const PositionControlInput& input, const Vector2& intermediatePoint) const {
    const auto startTrajectory = BBTrajectory2D(input.state.position, input.state.velocity, intermediatePoint, input.maxVel, ai::Constants::MAX_ACC_UPPER());
    int lowestCost = std::numeric_limits<int>::max();
    std::pair<int, BBTrajectory2D> bestEndTrajectory = {0, startTrajectory};

    for (int step = 0; PositionControlUtils::convertStepToTime(step) <= startTrajectory.getTotalTime(); step += 2) {
        const double startTrajectoryTime = PositionControlUtils::convertStepToTime(step);
        const auto state = StateVector{startTrajectory.getPosition(startTrajectoryTime), startTrajectory.getVelocity(startTrajectoryTime)};
        if (collisionDetector.doesCollideWithMovingObjects(state.position, input.robotId, input.avoidObjects, step)) {
            break;
        }

        const auto& [endTrajectory, collision] = trajectoryFromState(state, input);
        const int pathCost = scorePath(startTrajectoryTime, endTrajectory.getTotalTime(), collision);

        if (pathCost < lowestCost) {
            lowestCost = pathCost;
            bestEndTrajectory = {step, endTrajectory};
        }
    }

    return {
        .start = startTrajectory,
        .end = bestEndTrajectory.second,
        .cutoffIndex = bestEndTrajectory.first,
        .cost = lowestCost,
    };
}

std::pair<BBTrajectory2D, std::optional<double>> PathPlanning::trajectoryFromState(const StateVector& forState, const PositionControlInput& forInput) const {
    const auto trajectory = BBTrajectory2D(forState.position, forState.velocity, forInput.targetPos, forInput.maxVel, ai::Constants::MAX_ACC_UPPER());
    const auto steps = std::views::iota(0, PositionControlUtils::convertTimeToStep(trajectory.getTotalTime()));
    auto collision = std::find_if(steps.begin(), steps.end(), [&](int step) {
        const auto position = trajectory.getPosition(PositionControlUtils::convertStepToTime(step));
        return collisionDetector.doesCollideWithMovingObjects(position, forInput.robotId, forInput.avoidObjects, step) ||
               collisionDetector.doesCollideWithStaticObjects(position, forInput.avoidObjects);
    });

    return {trajectory, collision != steps.end() ? std::make_optional(PositionControlUtils::convertStepToTime(*collision)) : std::nullopt};
}

int PathPlanning::scorePath(double startTrajectoryDuration, double endTrajectoryDuration, std::optional<double> collisionTime) {
//    auto cost = PositionControlUtils::convertTimeToStep(startTrajectoryDuration + endTrajectoryDuration);
    int timeStepsCount = PositionControlUtils::convertTimeToStep(startTrajectoryDuration + endTrajectoryDuration);
    int cost = static_cast<int>(std::round(timeStepsCount * log(timeStepsCount)));
    if (collisionTime.has_value()) {
        cost += 500;  // Fixed penalty for collision
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
