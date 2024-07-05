#include "observer/filters/shot/ChipTrajectory.h"
#include <Eigen/Dense>

ChipTrajectory::ChipTrajectory(const Eigen::Vector3d& initialPos, const Eigen::Vector3d& initialVel, const Eigen::Vector2d& initialSpin, const BallParameters& ballParameters)
    : initialPos(initialPos), initialVel(initialVel), initialSpin(initialSpin), parameters(ballParameters) {
}

ChipTrajectory::ChipTrajectory(const Eigen::Vector3d& initialPos, const Eigen::Vector3d& initialVel, const BallParameters& ballParameters)
    : initialPos(initialPos), initialVel(initialVel), initialSpin(Eigen::Vector2d::Zero()), parameters(ballParameters) {
}

ChipTrajectory::ChipTrajectory(const Eigen::Vector2d& initialPos, const Eigen::Vector3d& initialVel, const BallParameters& ballParameters)
    : initialPos(initialPos.x(), initialPos.y(), 0), initialVel(initialVel), initialSpin(Eigen::Vector2d::Zero()), parameters(ballParameters) {
}

ShotState ChipTrajectory::getPositionAtTime(double time) {
    Eigen::Vector3d posNow = initialPos;
    Eigen::Vector3d velNow = initialVel;
    double tNow = 0.0;
    Eigen::Vector2d spin = initialSpin;
    // Only continue if hops are at least 0.01m, smaller hops is just rolling
    while ((velNow.z() * velNow.z()) / (2.0 * 9.81) > 0.01) {
        double tFly = 2.0 * velNow.z() / 9.81;
        if ((tNow + tFly) > time) {
            double t = time - tNow;
            posNow += velNow * t;
            posNow.z() += - 0.5 * 9.81 * t * t;
            velNow.z() -= 9.81 * t;
            return ShotState{posNow, velNow};
        }
        posNow += velNow * tFly;
        posNow.z() = 0;
        if (spin.norm() > 0) {
            velNow.x() = velNow.x() * parameters.getChipDampingXYOtherHops();
            velNow.y() = velNow.y() * parameters.getChipDampingXYOtherHops();
        } else {
            velNow.x() = velNow.x() * parameters.getChipDampingXYFirstHop();
            velNow.y() = velNow.y() * parameters.getChipDampingXYFirstHop();
        }
        velNow.z() = velNow.z() * parameters.getChipDampingZ();

        tNow += tFly;
        spin = velNow.head<2>() * (1.0 / parameters.getBallRadius());
    }
    velNow.z() = 0;
    double t = time - tNow;
    double tStop = -velNow.norm() / parameters.getAccRoll();
    if (t > tStop) t = tStop;
    posNow += velNow * t + velNow.normalized() * (0.5 * t * t * parameters.getAccRoll());
    velNow += velNow.normalized() * (t * parameters.getAccRoll());
    return ShotState{posNow, velNow};
}