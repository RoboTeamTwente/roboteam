#include "observer/filters/shot/KickTrajectory.h"
#include <cmath>
#include <Eigen/Dense>

KickTrajectory::KickTrajectory(const Eigen::Vector2d& initialPos, const Eigen::Vector2d& initialVel, const Eigen::Vector2d& initialSpin, const BallParameters& ballParameters)
    : initialPos(initialPos), initialVel(initialVel), initialSpin(initialSpin), parameters(ballParameters) {
    Eigen::Vector2d contactVelocity = initialVel - initialSpin * parameters.getBallRadius();
    // Ball is rolling
    if (contactVelocity.norm() < 0.01) {
        accSlide = initialVel.normalized() * parameters.getAccRoll();
        accSlideSpin = accSlide / parameters.getBallRadius();
        tSwitch = 0.0;
    // Ball is sliding
    } else {
        accSlide = contactVelocity.normalized() * parameters.getAccSlide();
        accSlideSpin = accSlide / (parameters.getBallRadius() * parameters.getInertiaDistribution());
        double f = 1.0 / (1.0 + 1.0 / parameters.getInertiaDistribution());
        Eigen::Vector2d slideVel = (initialSpin * parameters.getBallRadius() - initialVel) * f;

        if (std::fabs(accSlide.x()) > 0 && std::fabs(accSlide.x()) > std::fabs(accSlide.y())) {
            tSwitch = slideVel.x() / accSlide.x();
        } else if (std::fabs(accSlide.y()) > 0) {
            tSwitch = slideVel.y() / accSlide.y();
        } else {
            tSwitch = 0.0;
        }
        tSwitch = std::max(0.0, tSwitch);
    }
    velSwitch = initialVel + (accSlide * tSwitch);
    posSwitch = initialPos + (initialVel * tSwitch) + accSlide * (0.5 * tSwitch * tSwitch);
    accRoll = velSwitch.normalized() * parameters.getAccRoll();
}

KickTrajectory::KickTrajectory(const Eigen::Vector2d& initialPos, const Eigen::Vector2d& initialVel, const BallParameters& ballParameters)
    : KickTrajectory(initialPos, initialVel, Eigen::Vector2d::Zero(), ballParameters) {
}

KickTrajectory::KickTrajectory(const Eigen::Vector3d& initialPos, const Eigen::Vector3d& initialVel, const Eigen::Vector2d& initialSpin, const BallParameters& ballParameters)
    : KickTrajectory(Eigen::Vector2d(initialPos.head<2>()), Eigen::Vector2d(initialVel.head<2>()), initialSpin, ballParameters) {
}

ShotState KickTrajectory::getPositionAtTime(double time) {
    if (time < tSwitch) {
        auto pos = initialPos + initialVel * time + accSlide * (0.5 * time * time);
        auto vel = initialVel + accSlide * time;
        return ShotState({pos.x(), pos.y(), 0}, {vel.x(), vel.y(), 0});
    }
    auto t2 = time - tSwitch;
    if (time > getTimeAtRest()) {
        t2 = getTimeAtRest() - tSwitch;
    }
    auto pos = posSwitch + velSwitch * t2 + accRoll * (0.5 * t2 * t2);
    auto vel = velSwitch + accRoll * t2;
    return ShotState({pos.x(), pos.y(), 0}, {vel.x(), vel.y(), 0});
}

double KickTrajectory::getTimeAtRest() {
    auto tStop = -velSwitch.norm() / parameters.getAccRoll();
    return tSwitch + tStop;
}