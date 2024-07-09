#ifndef RTT_ROBOTEAM_OBSERVER_OBSERVER_SRC_FILTERS_VISION_ROBOT_ROBOTPOS_H_
#define RTT_ROBOTEAM_OBSERVER_OBSERVER_SRC_FILTERS_VISION_ROBOT_ROBOTPOS_H_

#include <roboteam_utils/Angle.h>

#include <Eigen/Dense>
#include <utility>

// TODO move to utils at some point
class RobotPos {
   public:
    RobotPos() = default;
    RobotPos(Eigen::Vector2d pos, double yaw) : position{std::move(pos)}, yaw{yaw} {}
    Eigen::Vector2d position;
    rtt::Angle yaw = 0.0;
};
class RobotVel {
   public:
    RobotVel() = default;
    RobotVel(Eigen::Vector2d velocity, double angularVelocity) : velocity{std::move(velocity)}, angularVelocity{angularVelocity} {}
    Eigen::Vector2d velocity;
    double angularVelocity = 0.0;
    RobotVel& operator+=(const RobotVel& other) {
        velocity += other.velocity;
        angularVelocity += other.angularVelocity;
        return *this;
    }
    RobotVel& operator/=(double scalar) {
        velocity /= scalar;
        angularVelocity /= scalar;
        return *this;
    }
};
struct RobotID {
    explicit RobotID(unsigned int id) : robotID{id} {}

    [[nodiscard]] bool isValid() const { return robotID < 16; }

    bool operator<=(const RobotID& other) const { return robotID <= other.robotID; }
    bool operator>=(const RobotID& other) const { return robotID >= other.robotID; }
    bool operator<(const RobotID& other) const { return robotID < other.robotID; }
    bool operator>(const RobotID& other) const { return robotID > other.robotID; }
    bool operator==(const RobotID& other) const { return robotID == other.robotID; }
    bool operator!=(const RobotID& other) const { return !(*this == other); }
    unsigned int robotID;
};
enum class TeamColor { BLUE, YELLOW };

struct TeamRobotID {
    TeamRobotID(unsigned int robotID, TeamColor color) : robot_id(robotID), team{color} {}
    bool operator==(const TeamRobotID& other) const { return robot_id == other.robot_id && team == other.team; }
    bool operator!=(const TeamRobotID& other) const { return !(*this == other); }
    bool operator<(const TeamRobotID& other) const {
        if (team == other.team) {
            return robot_id < other.robot_id;
        }
        return team == TeamColor::YELLOW;
    }
    bool operator>(const TeamRobotID& other) const {
        if (team == other.team) {
            return robot_id > other.robot_id;
        }
        return team == TeamColor::BLUE;
    }
    bool operator<=(const TeamRobotID& other) const { return *this < other || *this == other; }
    bool operator>=(const TeamRobotID& other) const { return *this > other || *this == other; }
    RobotID robot_id;
    TeamColor team;
};
#endif  // RTT_ROBOTEAM_OBSERVER_OBSERVER_SRC_FILTERS_VISION_ROBOT_ROBOTPOS_H_
