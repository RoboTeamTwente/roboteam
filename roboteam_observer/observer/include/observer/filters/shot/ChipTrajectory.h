#ifndef CHIP_TRAJECTORY_H
#define CHIP_TRAJECTORY_H

#include <Eigen/Dense>
#include <cmath>
#include "observer/filters/vision/ball/BallParameters.h"
#include "observer/filters/shot/ShotEvent.h"

class ChipTrajectory {
   private:
    Eigen::Vector3d initialPos;
    Eigen::Vector3d initialVel;
    Eigen::Vector2d initialSpin;
    BallParameters parameters;

   public:

    ChipTrajectory(const Eigen::Vector3d& initialPos, const Eigen::Vector3d& initialVel, const Eigen::Vector2d& initialSpin, const BallParameters& ballParameters);
    ChipTrajectory(const Eigen::Vector3d& initialPos, const Eigen::Vector3d& initialVel, const BallParameters& ballParameters);
    ChipTrajectory(const Eigen::Vector2d& initialPos, const Eigen::Vector3d& initialVel, const BallParameters& ballParameters);

    ShotState getPositionAtTime(double time);
};

#endif  // CHIP_TRAJECTORY_H