//
// Created by rolf on 24-06-21.
//

#ifndef RTT_ROBOTEAM_WORLD_OBSERVER_SRC_FILTERS_VISION_ROBOT_ROBOTPOS_H_
#define RTT_ROBOTEAM_WORLD_OBSERVER_SRC_FILTERS_VISION_ROBOT_ROBOTPOS_H_

#include <Eigen/Dense>
#include <utility>
#include <roboteam_utils/Angle.h>

//TODO move to utils at some point
class RobotPos {
 public:
  RobotPos() = default;
  RobotPos(Eigen::Vector2d pos, double angle) :
      position{std::move(pos)},
      angle{angle} {}
  Eigen::Vector2d position;
  rtt::Angle angle = 0.0;

};
class RobotVel {
 public:
  RobotVel() = default;
  RobotVel(Eigen::Vector2d velocity, double angularVelocity) :
      velocity{std::move(velocity)},
      angularVelocity{angularVelocity} {}
  Eigen::Vector2d velocity;
  double angularVelocity = 0.0;
  RobotVel& operator+=(const RobotVel& other){
    velocity+=other.velocity;
    angularVelocity+=other.angularVelocity;
    return *this;
  }
  RobotVel& operator/=(double scalar){
    velocity/=scalar;
    angularVelocity/=scalar;
    return *this;
  }

};
struct RobotID {
  RobotID(unsigned int id) : robotID{id}{}
  unsigned int robotID;
};
enum class TeamColor {
  BLUE,
  YELLOW
};

struct TeamRobotID {
  TeamRobotID(unsigned int robotID, TeamColor color) : robot_id(robotID), team{color}{

  }
  RobotID robot_id;
  TeamColor team;
};
#endif //RTT_ROBOTEAM_WORLD_OBSERVER_SRC_FILTERS_VISION_ROBOT_ROBOTPOS_H_
