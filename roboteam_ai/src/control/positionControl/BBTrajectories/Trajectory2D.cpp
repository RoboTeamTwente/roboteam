#include "control/positionControl/BBTrajectories/Trajectory2D.h"

#include <cmath>
#include <ruckig/ruckig.hpp>

#include "control/positionControl/BBTrajectories/BBTrajectory2D.h"
#include "roboteam_utils/Print.h"

using namespace ruckig;
namespace rtt {
std::unordered_map<int, Vector2> Trajectory2D::lastAcceleration;

Vector2 Trajectory2D::getLastAcceleration(int robotId) {
    if (Trajectory2D::lastAcceleration.contains(robotId)) {
        return Trajectory2D::lastAcceleration[robotId];
    } else {
        return Vector2(0, 0);
    }
}
Trajectory2D::Trajectory2D(const Vector2 &initialPos, const Vector2 &initialVel, const Vector2 &finalPos, double maxVel, double maxAcc, double maxJerk, int robotId)
    : Trajectory2D(initialPos, initialVel, getLastAcceleration(robotId), finalPos, Vector2(0, 0), maxVel, maxAcc, maxJerk) {}

Trajectory2D::Trajectory2D(const Vector2 &initialPos, const Vector2 &initialVel, const Vector2 &finalPos, const Vector2 &finalVel, double maxVel, double maxAcc, double maxJerk,
                           int robotId)
    : Trajectory2D(initialPos, initialVel, getLastAcceleration(robotId), finalPos, finalVel, maxVel, maxAcc, maxJerk) {}

Trajectory2D::Trajectory2D(const Vector2 &initialPos, const Vector2 &initialVel, const Vector2 &initialAcc, const Vector2 &finalPos, double maxVel, double maxAcc, double maxJerk)
    : Trajectory2D(initialPos, initialVel, initialAcc, finalPos, Vector2(0, 0), maxVel, maxAcc, maxJerk) {}

Trajectory2D::Trajectory2D(const Vector2 &initialPos, const Vector2 &initialVel, const Vector2 &initialAcc, const Vector2 &finalPos, const Vector2 &finalVel, double maxVel,
                           double maxAcc, double maxJerk) {
    // The idea is to do a binary search over alpha to find a trajectory in x and y direction (which is minimal time)
    double inc = M_PI_4 * 0.5;
    double alpha = M_PI_4;
    constexpr double iterationLimit = 1e-7;
    constexpr double timeDiffLimit = 1e-3;
    while (inc > iterationLimit) {
        InputParameter<1> inputX;
        inputX.current_position[0] = initialPos.x;
        inputX.current_velocity[0] = initialVel.x;
        inputX.current_acceleration[0] = initialAcc.x;
        inputX.target_position[0] = finalPos.x;
        inputX.target_velocity[0] = finalVel.x;
        inputX.target_acceleration[0] = 0;
        inputX.max_velocity[0] = std::max(maxVel * cos(alpha), std::abs(finalVel.x));
        inputX.max_acceleration[0] = maxAcc * cos(alpha);
        inputX.max_jerk[0] = maxJerk * cos(alpha);
        Ruckig<1> ruckigX;
        ruckigX.calculate(inputX, x);

        InputParameter<1> inputY;
        inputY.current_position[0] = initialPos.y;
        inputY.current_velocity[0] = initialVel.y;
        inputY.current_acceleration[0] = initialAcc.y;
        inputY.target_position[0] = finalPos.y;
        inputY.target_velocity[0] = finalVel.y;
        inputY.target_acceleration[0] = 0;
        inputY.max_velocity[0] = std::max(maxVel * sin(alpha), std::abs(finalVel.y));
        inputY.max_acceleration[0] = maxAcc * sin(alpha);
        inputY.max_jerk[0] = maxJerk * sin(alpha);
        Ruckig<1> ruckigY;
        ruckigY.calculate(inputY, y);

        double diff = abs(x.get_duration() - y.get_duration());
        // If the trajectories match enough we stop earlier
        if (diff < timeDiffLimit) {
            return;
        }
        if (x.get_duration() > y.get_duration()) {
            alpha -= inc;
        } else {
            alpha += inc;
        }
        inc *= 0.5;
    }

    // rtt::ai::control::BBTrajectory2D BBTNoCollision = rtt::ai::control::BBTrajectory2D(initialPos, initialVel, finalPos, maxVel, maxAcc);
    // std::pair<std::vector<rtt::ai::control::BBTrajectoryPart>, std::vector<rtt::ai::control::BBTrajectoryPart>> parts = BBTNoCollision.getParts();
    // x.parts = parts.first;
    // x.finalPos = finalPos.x;
    // y.parts = parts.second;
    // y.finalPos = finalPos.y;
}

void Trajectory2D::addTrajectory(const Trajectory2D &extraTrajectory, double addFromTime) {
    xAdded = extraTrajectory.x;
    yAdded = extraTrajectory.y;
    addedFromTime = addFromTime;
    // x.addTrajectory(extraTrajectory.x.parts, addFromTime);
    // x.finalPos = extraTrajectory.x.finalPos;
    // y.addTrajectory(extraTrajectory.y.parts, addFromTime);
    // y.finalPos = extraTrajectory.y.finalPos;
}

std::vector<Vector2> Trajectory2D::getPathApproach(double timeStep) const {
    std::vector<Vector2> points;
    auto totalTime = getTotalTime();
    double time = 0;

    if (totalTime == std::numeric_limits<double>::infinity()) {
        RTT_ERROR("Infinite while loop")
        // throw std::runtime_error("Total time of infinity!");
        return {getPosition(0)};
    }

    while (time <= totalTime) {
        time += timeStep;
        points.push_back(getPosition(time));
    }
    return points;
}

std::vector<Vector2> Trajectory2D::getVelocityVector(double timeStep) const {
    std::vector<Vector2> velocities;
    auto totalTime = getTotalTime();
    double time = 0;

    if (totalTime == std::numeric_limits<double>::infinity()) {
        RTT_ERROR("Infinite while loop")
        // throw std::runtime_error("Total time of infinity!");
        return {getPosition(0)};
    }

    while (time <= totalTime) {
        time += timeStep;
        velocities.push_back(getVelocity(time));
    }
    return velocities;
}

Vector2 Trajectory2D::getPosition(double t) const {
    double newPosX, newVelX, newAccX, newPosY, newVelY, newAccY;

    if (xAdded && t > addedFromTime) {
        xAdded->at_time(t, newPosX, newVelX, newAccX);
    } else {
        x.at_time(t, newPosX, newVelX, newAccX);
    }

    if (yAdded && t > addedFromTime) {
        yAdded->at_time(t, newPosY, newVelY, newAccY);
    } else {
        y.at_time(t, newPosY, newVelY, newAccY);
    }

    return Vector2(newPosX, newPosY);
}

Vector2 Trajectory2D::getVelocity(double t) const {
    double newPosX, newVelX, newAccX, newPosY, newVelY, newAccY;

    if (xAdded && t > addedFromTime) {
        xAdded->at_time(t, newPosX, newVelX, newAccX);
    } else {
        x.at_time(t, newPosX, newVelX, newAccX);
    }

    if (yAdded && t > addedFromTime) {
        yAdded->at_time(t, newPosY, newVelY, newAccY);
    } else {
        y.at_time(t, newPosY, newVelY, newAccY);
    }

    return Vector2(newVelX, newVelY);
}

Vector2 Trajectory2D::getAcceleration(double t) const {
    double newPosX, newVelX, newAccX, newPosY, newVelY, newAccY;

    if (xAdded && t > addedFromTime) {
        xAdded->at_time(t, newPosX, newVelX, newAccX);
    } else {
        x.at_time(t, newPosX, newVelX, newAccX);
    }

    if (yAdded && t > addedFromTime) {
        yAdded->at_time(t, newPosY, newVelY, newAccY);
    } else {
        y.at_time(t, newPosY, newVelY, newAccY);
    }

    return Vector2(newAccX, newAccY);
}

double Trajectory2D::getTotalTime() const { return std::max(x.get_duration(), y.get_duration()); }

}  // namespace rtt