//
// Created by rolf on 24-06-21.
//

#ifndef RTT_ROBOTEAM_WORLD_OBSERVER_SRC_FILTERS_VISION_ROBOT_FILTEREDROBOT_H_
#define RTT_ROBOTEAM_WORLD_OBSERVER_SRC_FILTERS_VISION_ROBOT_FILTEREDROBOT_H_

#include <utility>

#include "RobotPos.h"
struct FilteredRobot {
  explicit FilteredRobot(TeamRobotID id,
                RobotPos position,
                RobotVel velocity,
                double health,
                double posUncertainty,
                double velocityUncertainty,
                double angleUncertainty,
                double angularVelUncertainty) :
      id{id},
      position{std::move(position)},
      velocity{std::move(velocity)},
      health{health},
      posUncertainty{posUncertainty},
      velocityUncertainty{velocityUncertainty},
      angleUncertainty{angleUncertainty},
      angularVelUncertainty{angularVelUncertainty} {
  }
  TeamRobotID id;
  RobotPos position;
  RobotVel velocity;
  double health;
  double posUncertainty;
  double velocityUncertainty;
  double angleUncertainty;
  double angularVelUncertainty;
};

#endif //RTT_ROBOTEAM_WORLD_OBSERVER_SRC_FILTERS_VISION_ROBOT_FILTEREDROBOT_H_
