#ifndef KICKESTIMATOR_H
#define KICKESTIMATOR_H
#include <dlib/optimization.h>

#include <vector>

#include "ShotEvent.h"
#include "observer/filters/shot/KickTrajectory.h"
#include "observer/filters/vision/ball/BallParameters.h"
#include "observer/filters/vision/ball/FilteredBall.h"

class KickEstimator {
   private:
    std::vector<BallObservation> ballsSinceShot;
    ShotEvent shotEvent;
    BallParameters ballParameters;
    int pruneIndex = 1;

   public:
    Eigen::Vector3d bestShotPos;
    Eigen::Vector3d bestShotVel;
    KickEstimator(const ShotEvent& shotEvent, const BallParameters& ballParameters);
    double getAverageDistance();
    double getAverageDistance(Eigen::Vector3d shotPos, Eigen::Vector3d shotVel, Eigen::Vector2d shotSpin);
    std::pair<Eigen::Vector3d, Eigen::Vector3d> slidingBall();
    Eigen::Vector2d getKickDir();
    std::pair<Eigen::Vector3d, Eigen::Vector3d> noSpinBall();
    void addFilteredBall(const BallObservation& newBall);
};

#endif  // KICKESTIMATOR_H