#ifndef RTT_ROBOTEAM_OBSERVER_OBSERVER_SRC_FILTERS_VISION_BALL_FILTEREDBALL_H_
#define RTT_ROBOTEAM_OBSERVER_OBSERVER_SRC_FILTERS_VISION_BALL_FILTEREDBALL_H_

#include <proto/WorldBall.pb.h>

#include <Eigen/Dense>
struct FilteredBall {
    [[nodiscard]] proto::WorldBall asWorldBall() const;
    FilteredBall(Eigen::Vector2d pos, Eigen::Vector2d vel, double health, double posUncertainty, double velocityUncertainty);
    FilteredBall() = default;
    Eigen::Vector2d position;
    Eigen::Vector2d velocity;
    double health;
    double posUncertainty;
    double velocityUncertainty;
};

#endif  // RTT_ROBOTEAM_OBSERVER_OBSERVER_SRC_FILTERS_VISION_BALL_FILTEREDBALL_H_
