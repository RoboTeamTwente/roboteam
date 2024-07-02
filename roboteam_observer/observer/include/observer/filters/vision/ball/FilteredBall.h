#ifndef RTT_ROBOTEAM_OBSERVER_OBSERVER_SRC_FILTERS_VISION_BALL_FILTEREDBALL_H_
#define RTT_ROBOTEAM_OBSERVER_OBSERVER_SRC_FILTERS_VISION_BALL_FILTEREDBALL_H_

#include <proto/WorldBall.pb.h>
#include <roboteam_utils/Time.h>

#include <Eigen/Dense>
#include <optional>

#include "BallObservation.h"
struct FilteredBall {
    [[nodiscard]] proto::WorldBall asWorldBall() const;
    FilteredBall(Eigen::Vector2d pos, Eigen::Vector2d vel, Time time, Eigen::Vector2d positionCamera, std::optional<BallObservation> currentObservation);
    FilteredBall() = default;
    Eigen::Vector2d position;
    Eigen::Vector2d velocity;
    Time time;
    Eigen::Vector2d positionCamera;
    std::optional<BallObservation> currentObservation;
};

#endif  // RTT_ROBOTEAM_OBSERVER_OBSERVER_SRC_FILTERS_VISION_BALL_FILTEREDBALL_H_