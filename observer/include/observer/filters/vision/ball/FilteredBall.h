//
// Created by rolf on 05-07-21.
//

#ifndef RTT_ROBOTEAM_WORLD_OBSERVER_SRC_FILTERS_VISION_BALL_FILTEREDBALL_H_
#define RTT_ROBOTEAM_WORLD_OBSERVER_SRC_FILTERS_VISION_BALL_FILTEREDBALL_H_

#include <Eigen/Dense>
struct FilteredBall {

  Eigen::Vector2d position;
  Eigen::Vector2d velocity;
  double health;
  double posUncertainty;
  double velocityUncertainty;
};

#endif //RTT_ROBOTEAM_WORLD_OBSERVER_SRC_FILTERS_VISION_BALL_FILTEREDBALL_H_
