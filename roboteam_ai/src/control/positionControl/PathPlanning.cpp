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

PathPlanning::PathPlanning(const CollisionDetector& collisionDetector) : collisionDetector(collisionDetector), fieldWidth(0), fieldHeight(0) {}

void PathPlanning::generateNewPath(std::vector<StateVector>& pathBuffer, const PositionControlInput& input) const noexcept {
    const auto& [directTrajectory, directTrajectoryCollision] = calculateTrajectoryWithCollisionDetection(0, input.state, input);

    // If the direct trajectory is collision free, we can just use that.
    if (!directTrajectoryCollision.has_value()) {
        fillPathBuffer(pathBuffer, directTrajectory);
        return;
    }

    //TODO: Does this produces better results than the old method? (input.state.position)?
    // fieldHeight vs fieldWidth?
    auto firstCollision = directTrajectory.getPosition(directTrajectoryCollision.value());
    double pointExtension = fieldHeight / 18;  // How far the pointToDrawFrom has to be from the obstaclePosition
    Vector2 pointToDrawFrom = firstCollision + (firstCollision - input.targetPos).normalize() * pointExtension;



    int directTrajectoryCost = scorePath(directTrajectory.getTotalTime(), 0, directTrajectoryCollision);
    const auto points = generateIntermediatePoints(pointToDrawFrom);
    auto trajectories = std::array<ScoredTrajectoryPair, POINTS_COUNT>();

    std::transform(points.begin(), points.end(), trajectories.begin(), [&](const Vector2& point) {
        interface::Input::drawData(interface::Visual::PATHFINDING_DEBUG, {point}, Qt::yellow, input.robotId, interface::Drawing::CROSSES);
        return searchBestTrajectoryToIntermediatePoint(directTrajectory, directTrajectoryCost, input, point);
    });

    const auto bestTrajectory = std::min_element(trajectories.begin(), trajectories.end(), [](const auto a, const auto b) { return a.cost < b.cost; });
    fillPathBuffer(pathBuffer, *bestTrajectory);
    if (bestTrajectory->cost < 1500 || input.maxVel <= 0.3) {
        fillPathBuffer(pathBuffer, *bestTrajectory);
    } else {
        RTT_WARNING("No good path found, limiting maxVel to half (", input.robotId, ",", input.maxVel / 2, ")");
        PositionControlInput newInput = {
            .robotId = input.robotId,
            .state = input.state,
            .targetPos = input.targetPos,
            .maxVel = 0.1,
            .avoidObjects = input.avoidObjects,
        };
        generateNewPath(pathBuffer, newInput);
//        const auto& [directTrajectory2, directTrajectoryCollision2] = calculateTrajectoryWithCollisionDetection(0, newInput.state, newInput);
//        fillPathBuffer(pathBuffer, bestTrajectory);
    }
}

std::array<Vector2, PathPlanning::POINTS_COUNT> PathPlanning::generateIntermediatePoints(const Vector2& center) const {
    auto intermediatePoints = std::array<Vector2, POINTS_COUNT>();
    for (int radiusOffset = 0; radiusOffset < static_cast<int>(RADIUS_OFFSETS.size()); radiusOffset++) {
        auto pointToRotate = (
            center + Vector2{0, fieldHeight / RADIUS_OFFSETS[radiusOffset]}
        ).rotateAroundPoint(radiusOffset * ROTATE_OFFSET, center);

        for (int pointOffset = 0; pointOffset < POINTS_PER_CIRCLE; pointOffset++) {
            intermediatePoints[radiusOffset * POINTS_PER_CIRCLE + pointOffset] = (pointToRotate.rotateAroundPoint(pointOffset * ANGLE_BETWEEN_POINTS, center));
        }
    }

    return intermediatePoints;
}

ScoredTrajectoryPair PathPlanning::searchBestTrajectoryToIntermediatePoint(const BBTrajectory2D& directTrajectory, int directTrajectoryCost, const PositionControlInput& input,
                                                                           const Vector2& intermediatePoint) const {
    // The best default trajectory is the direct trajectory.
    // Cutoff  0 means that the whole part2 trajectory is used.
    auto scoredTrajectory = ScoredTrajectoryPair{.part1 = directTrajectory, .part2 = directTrajectory, .part1CutoffIndex = 0, .cost = directTrajectoryCost};

    const auto part1 = BBTrajectory2D(input.state.position, input.state.velocity, intermediatePoint, input.maxVel, ai::Constants::MAX_ACC_UPPER());
    for (int step = 0; PositionControlUtils::convertStepToTime(step) <= part1.getTotalTime(); step += 2) {
        const double startTrajectoryTime = PositionControlUtils::convertStepToTime(step);
        const auto state = StateVector{part1.getPosition(startTrajectoryTime), part1.getVelocity(startTrajectoryTime)};

        // If the start trajectory is already colliding, we can stop searching.
        if (collisionDetector.doesCollideWithMovingObjects(state, input.robotId, input.avoidObjects, step)) {
            break;
        }

        const auto [part2, collision] = calculateTrajectoryWithCollisionDetection(step, state, input);
        const int pathCost = scorePath(startTrajectoryTime, part2.getTotalTime(), collision);

        if (pathCost < scoredTrajectory.cost) {
            scoredTrajectory = {
                .part1 = part1,
                .part2 = part2,
                .part1CutoffIndex = step,
                .cost = pathCost,
            };
        }

        // TODO: Would this work?
        //        if (!collision.has_value()) { break; }
    }

    return scoredTrajectory;
}

std::pair<BBTrajectory2D, std::optional<double>> PathPlanning::calculateTrajectoryWithCollisionDetection(int stepOffset, const StateVector& forState,
                                                                                                         const PositionControlInput& forInput) const {
    const auto trajectory = BBTrajectory2D(forState.position, forState.velocity, forInput.targetPos, forInput.maxVel, ai::Constants::MAX_ACC_UPPER());
    const auto steps = std::views::iota(0, PositionControlUtils::convertTimeToStep(trajectory.getTotalTime()));
    auto collision = std::find_if(steps.begin(), steps.end(), [&](int step) {
        const double time = PositionControlUtils::convertStepToTime(step);
        const auto position = trajectory.getPosition(time);
        const auto velocity = trajectory.getVelocity(time);
        return collisionDetector.doesCollideWithMovingObjects({position, velocity}, forInput.robotId, forInput.avoidObjects, stepOffset + step) ||
               collisionDetector.doesCollideWithStaticObjects(position, forInput.avoidObjects);
    });

    return {trajectory, collision != steps.end() ? std::make_optional(PositionControlUtils::convertStepToTime(*collision)) : std::nullopt};
}

int PathPlanning::scorePath(double part1Duration, double part2Duration, std::optional<double> collisionTime) {
    int timeStepsCount = PositionControlUtils::convertTimeToStep(part1Duration + part2Duration);

    // Cost is n * log(n) proportional to the path length => shorter paths are preferred.
    int cost = static_cast<int>(std::round(timeStepsCount * log(timeStepsCount)));

    if (collisionTime.has_value()) {
        cost += 1500;  // Fixed penalty for collision
        cost -= static_cast<int>(std::round(part1Duration + collisionTime.value()));
    }
    return cost;
}

void PathPlanning::fillPathBuffer(std::vector<StateVector>& pathBuffer, const ScoredTrajectoryPair& trajectoryPair) {


    auto steps = std::views::iota(0, trajectoryPair.part1CutoffIndex);
    for (int step : steps) {
        const auto time = PositionControlUtils::convertStepToTime(step);
        auto vel = trajectoryPair.part1.getVelocity(time);
        pathBuffer.emplace_back(StateVector{trajectoryPair.part1.getPosition(time), trajectoryPair.cost >= 1500 ? vel.stretchToLength(0.3) : vel});
    }

    steps = std::views::iota(0, PositionControlUtils::convertTimeToStep(trajectoryPair.part2.getTotalTime()));
    for (int step : steps) {
        const auto time = PositionControlUtils::convertStepToTime(step);
        auto vel = trajectoryPair.part2.getVelocity(time);
        pathBuffer.emplace_back(StateVector{trajectoryPair.part2.getPosition(time), trajectoryPair.cost >= 1500 ? vel.stretchToLength(0.3) : vel});
    }
}

void PathPlanning::fillPathBuffer(std::vector<StateVector>& pathBuffer, const BBTrajectory2D& trajectory) {
    const auto steps = std::views::iota(0, PositionControlUtils::convertTimeToStep(trajectory.getTotalTime()));
    for (int step : steps) {
        const auto time = PositionControlUtils::convertStepToTime(step);
        pathBuffer.emplace_back(StateVector{trajectory.getPosition(time), trajectory.getVelocity(time)});
    }
}

void PathPlanning::updateConstraints(const rtt::Field& field) {
    fieldWidth = field.playArea.width();
    fieldHeight = field.playArea.height();
}

}  // namespace rtt::ai::control
#endif  // RTT_PATHPLANNING_H
