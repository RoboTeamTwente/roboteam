#ifndef RTT_ROBOTEAM_OBSERVER_OBSERVER_SRC_FILTERS_VISION_BALL_CAMERABALLFILTER_H_
#define RTT_ROBOTEAM_OBSERVER_OBSERVER_SRC_FILTERS_VISION_BALL_CAMERABALLFILTER_H_
#include <optional>

#include "BallObservation.h"
#include "FilteredBall.h"
#include "GroundBallExtendedKalmanFilter.h"
#include "observer/filters/vision/CameraObjectFilter.h"
#include "observer/filters/vision/robot/RobotFilter.h"

struct CameraGroundBallPrediction {
    CameraGroundBallPrediction() = default;
    CameraGroundBallPrediction(Eigen::Vector2d pos, Eigen::Vector2d vel, Time time);
    Eigen::Vector2d position;
    Eigen::Vector2d velocity;
    Time time;
};
struct CameraGroundBallPredictionObservationPair {
    CameraGroundBallPrediction prediction;
    std::optional<BallObservation> observation;
};
class CameraGroundBallFilter : public CameraObjectFilter {
   public:
    explicit CameraGroundBallFilter(const BallObservation& observation, const Eigen::Vector2d& velocity_estimate = Eigen::Vector2d::Zero());
    [[nodiscard]] FilteredBall getEstimate(Time time) const;
    [[nodiscard]] Eigen::Vector2d getVelocityEstimate(Time time) const;

    [[nodiscard]] CameraGroundBallPrediction predict(Time time, std::vector<FilteredRobot> yellowRobots, std::vector<FilteredRobot> blueRobots);

    bool processDetections(const CameraGroundBallPredictionObservationPair& prediction_observation_pair);

   private:
    bool checkRobots(const std::vector<FilteredRobot>& robots, const Eigen::Vector2d& positionEstimate, const Eigen::Vector2d& velocityEstimate);
    bool checkRobotCollision(const FilteredRobot& robot, const Eigen::Vector2d& positionEstimate, const Eigen::Vector2d& velocityEstimate);
    void predictFilter(const CameraGroundBallPrediction& prediction);
    void update(const BallObservation& observation);
    bool updateNotSeen(Time time);
    GroundBallExtendedKalmanFilter ekf;
};

#endif  // RTT_ROBOTEAM_OBSERVER_OBSERVER_SRC_FILTERS_VISION_BALL_CAMERABALLFILTER_H_
