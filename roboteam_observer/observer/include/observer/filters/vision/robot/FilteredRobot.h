#ifndef RTT_ROBOTEAM_OBSERVER_OBSERVER_SRC_FILTERS_VISION_ROBOT_FILTEREDROBOT_H_
#define RTT_ROBOTEAM_OBSERVER_OBSERVER_SRC_FILTERS_VISION_ROBOT_FILTEREDROBOT_H_

#include <proto/WorldRobot.pb.h>

#include "RobotPos.h"
struct FilteredRobot {
    explicit FilteredRobot(TeamRobotID id, RobotPos position, RobotVel velocity, double health, double posUncertainty, double velocityUncertainty, double yawUncertainty,
                           double angularVelUncertainty)
        : id{id},
          position{std::move(position)},
          velocity{std::move(velocity)},
          health{health},
          posUncertainty{posUncertainty},
          velocityUncertainty{velocityUncertainty},
          yawUncertainty{yawUncertainty},
          angularVelUncertainty{angularVelUncertainty} {}
    [[nodiscard]] proto::WorldRobot asWorldRobot() const {
        proto::WorldRobot robot;
        robot.mutable_pos()->set_x(position.position.x());
        robot.mutable_pos()->set_y(position.position.y());
        robot.set_yaw(position.yaw);
        robot.mutable_vel()->set_x(velocity.velocity.x());
        robot.mutable_vel()->set_y(velocity.velocity.y());
        robot.set_w(velocity.angularVelocity);
        robot.set_id(id.robot_id.robotID);
        return robot;
    }
    TeamRobotID id;
    RobotPos position;
    RobotVel velocity;
    double health;
    double posUncertainty;
    double velocityUncertainty;
    double yawUncertainty;
    double angularVelUncertainty;
};

#endif  // RTT_ROBOTEAM_OBSERVER_OBSERVER_SRC_FILTERS_VISION_ROBOT_FILTEREDROBOT_H_
