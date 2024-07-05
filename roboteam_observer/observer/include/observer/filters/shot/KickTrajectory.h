#ifndef KICK_TRAJECTORY_H
#define KICK_TRAJECTORY_H

#include <Eigen/Dense>
#include <cmath>
#include "observer/filters/vision/ball/BallParameters.h"
#include "observer/filters/shot/ShotEvent.h"

class KickTrajectory {
   private:
    double tSwitch;
    Eigen::Vector2d posSwitch;
    Eigen::Vector2d velSwitch;
    Eigen::Vector2d accSlide;
    Eigen::Vector2d accSlideSpin;
    Eigen::Vector2d accRoll;
    Eigen::Vector2d initialPos;
    Eigen::Vector2d initialVel;
    Eigen::Vector2d initialSpin;
    BallParameters parameters;

   public:
    KickTrajectory(const Eigen::Vector2d& initialPos, const Eigen::Vector2d& initialVel, const Eigen::Vector2d& initialSpin, const BallParameters& ballParameters);
    KickTrajectory(const Eigen::Vector2d& initialPos, const Eigen::Vector2d& initialVel, const BallParameters& ballParameters);
    KickTrajectory(const Eigen::Vector3d& initialPos, const Eigen::Vector3d& initialVel, const Eigen::Vector2d& initialSpin, const BallParameters& ballParameters);

    ShotState getPositionAtTime(double time);
    double getTimeAtRest();
};

#endif  // KICK_TRAJECTORY_H