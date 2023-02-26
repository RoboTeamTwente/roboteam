//
// Created by rolf on 26-09-20.
//

#include "control/positionControl/BBTrajectories/BBTrajectory2D.h"

#include <cmath>

#include "control/positionControl/PositionControlUtils.h"
#include "utilities/Constants.h"
namespace rtt::ai::control {

BBTrajectory2D::BBTrajectory2D(const Vector2 &initialPos, const Vector2 &initialVel, const Vector2 &finalPos, double maxVel, double maxAcc) {
    generateSyncedTrajectory(initialPos, initialVel, finalPos, maxVel, maxAcc);
}
void BBTrajectory2D::generateTrajectory(const Vector2 &initialPos, const Vector2 &initialVel, const Vector2 &finalPos, double maxVel, double maxAcc, double alpha) {
    x = BBTrajectory1D(initialPos.x, initialVel.x, finalPos.x, maxVel * cos(alpha), maxAcc * cos(alpha));
    y = BBTrajectory1D(initialPos.y, initialVel.y, finalPos.y, maxVel * sin(alpha), maxAcc * sin(alpha));
}

void BBTrajectory2D::generateSyncedTrajectory(const Vector2 &initialPos, const Vector2 &initialVel, const Vector2 &finalPos, double maxVel, double maxAcc) {
    // The idea is to do a binary search over alpha to find a trajectory in x and y direction (which is minimal time)
    double inc = M_PI_4 * 0.5;
    double alpha = M_PI_4;
    // TODO: tune convergence numbers
    constexpr double iterationLimit = 1e-7;
    constexpr double timeDiffLimit = 0.001;
    while (inc > iterationLimit) {
        generateTrajectory(initialPos, initialVel, finalPos, maxVel, maxAcc, alpha);
        double diff = abs(x.getTotalTime() - y.getTotalTime());
        // If the trajectories match enough we stop earlier
        if (diff < timeDiffLimit) {
            return;
        }
        if (x.getTotalTime() > y.getTotalTime()) {
            alpha -= inc;
        } else {
            alpha += inc;
        }
        inc *= 0.5;
    }
}

Vector2 BBTrajectory2D::getPosition(double t) const { return Vector2(x.getPosition(t), y.getPosition(t)); }

Vector2 BBTrajectory2D::getVelocity(double t) const { return Vector2(x.getVelocity(t), y.getVelocity(t)); }

Vector2 BBTrajectory2D::getAcceleration(double t) const { return Vector2(x.getAcceleration(t), y.getAcceleration(t)); }

[[maybe_unused]] double BBTrajectory2D::getTotalTime() const { return std::max(x.getTotalTime(), y.getTotalTime()); }

std::pair<std::vector<BBTrajectoryPart>, std::vector<BBTrajectoryPart>> BBTrajectory2D::getParts() { return std::make_pair(x.getParts(), y.getParts()); }


BBTrajectory2D::Iterator::Iterator(const BBTrajectory2D &trajectory, double time): trajectory(trajectory), time(time) {}

BBTrajectory2D::Iterator::reference BBTrajectory2D::Iterator::operator*() {
    if (!item.has_value()) {
        item = {{ time, {trajectory.getPosition(time), trajectory.getVelocity(time) }}};
    }
    return item.value();
}

BBTrajectory2D::Iterator::pointer BBTrajectory2D::Iterator::operator->() {
    if (!item.has_value()) {
        item = {{ time, {trajectory.getPosition(time), trajectory.getVelocity(time) }}};
    }

    return &item.value();
}

bool BBTrajectory2D::Iterator::operator==(const BBTrajectory2D::Iterator& other) const {
    return abs(other.time - time) < PositionControlUtils::TIME_STEP;
}

bool BBTrajectory2D::Iterator::operator!=(const BBTrajectory2D::Iterator& other) const {
    return !(*this == other);
}

BBTrajectory2D::Iterator& BBTrajectory2D::Iterator::operator++() {
    time += PositionControlUtils::TIME_STEP;
    item = std::nullopt;
    return *this;
}

}  // namespace rtt::BB