#ifndef CHIPESTIMATOR_H
#define CHIPESTIMATOR_H

#include <Eigen/Dense>
#include <iostream>
#include <tuple>
#include <vector>

#include "ShotEvent.h"
#include "observer/filters/shot/ChipTrajectory.h"
#include "observer/filters/vision/ball/BallParameters.h"
#include "observer/filters/vision/ball/FilteredBall.h"

class ChipEstimator {
   private:
    std::vector<BallObservation> ballsSinceShot;
    ShotEvent shotEvent;
    Eigen::Vector3d bestShotPos;
    Eigen::Vector3d bestShotVel;
    BallParameters ballParameters;
    CameraMap cameraMap;
    int pruneIndex = 1;
    bool doFirstHop = true;
    double bestTOff = 0.0;

   public:
    ChipEstimator(const ShotEvent& shotEvent, const BallParameters& ballParameters, const CameraMap& cameraMap);
    double getAverageDistance();
    double getAverageDistance(Eigen::Vector3d shotPos, Eigen::Vector3d shotVel, Eigen::Vector2d shotSpin);
    void addFilteredBall(const BallObservation& newBall);
    std::pair<Eigen::Vector3d, double> noSpinBall(double tOff);
    Eigen::Vector3d getBestShotVel() { return bestShotVel; }
    Eigen::Vector3d getBestShotPos() { return bestShotPos; }
    Time getFirstBallTime() { return ballsSinceShot[0].timeCaptured; }
};

#endif  // CHIPESTIMATOR_H