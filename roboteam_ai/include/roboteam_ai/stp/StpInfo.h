#ifndef RTT_STPINFO_H
#define RTT_STPINFO_H

#include <optional>
#include <roboteam_utils/Field.hpp>
#include <roboteam_utils/RobotCommands.hpp>

#include "computations/PositionScoring.h"
#include "utilities/StpInfoEnums.h"
#include "world/views/BallView.hpp"
#include "world/views/RobotView.hpp"

namespace rtt::ai::stp {
namespace world = ::rtt::world;

/**
 * StpInfo bundles all info a robot could need in one struct
 * This data propagates all the way from plays down to skills
 */
struct StpInfo {
   public:
    const std::optional<world::view::RobotView>& getRobot() const { return robot; }
    void setRobot(const std::optional<world::view::RobotView>& robot) { this->robot = robot; }

    const std::optional<Field>& getField() const { return field; }
    void setField(const std::optional<Field>& field) { this->field = field; }

    const std::optional<world::view::BallView>& getBall() const { return ball; }
    void setBall(const std::optional<world::view::BallView>& ball) { this->ball = ball; }

    const std::optional<Vector2>& getPositionToMoveTo() const { return positionToMoveTo; }
    void setPositionToMoveTo(const std::optional<Vector2>& position) { this->positionToMoveTo = position; }
    void setPositionToMoveTo(const std::optional<gen::ScoredPosition>& scoredPosition) { setPositionToMoveTo(scoredPosition->position); }

    const Vector2& getTargetVelocity() const { return targetVelocity; }
    void setTargetVelocity(const Vector2& targetVelocity) { this->targetVelocity = targetVelocity; }

    const double getMaxJerk() const { return maxJerk; }
    void setMaxJerk(double maxJerk) { this->maxJerk = maxJerk; }

    const std::optional<Vector2>& getPositionToShootAt() const { return positionToShootAt; }
    void setPositionToShootAt(const std::optional<Vector2>& position) { this->positionToShootAt = position; }

    const std::optional<Vector2>& getPositionToDefend() const { return positionToDefend; }
    void setPositionToDefend(const std::optional<Vector2>& position) { this->positionToDefend = position; }

    double getKickChipVelocity() const { return kickChipVelocity; }
    void setKickChipVelocity(double kickChipVelocity) { this->kickChipVelocity = kickChipVelocity; }

    Angle getYaw() const { return yaw; }
    void setYaw(double yaw) { this->yaw = Angle(yaw); }

    bool getDribblerOn() const { return dribblerOn; }
    void setDribblerOn(bool dribblerOn) { this->dribblerOn = dribblerOn; }

    ShotPower getShotPower() const { return shotPower; }
    void setShotPower(ShotPower shotPower) { this->shotPower = shotPower; }

    rtt::KickType getKickOrChip() const { return kickOrChip; }
    void setKickOrChip(rtt::KickType kickOrChip) { this->kickOrChip = kickOrChip; }

    world::World* getCurrentWorld() const { return currentWorld; }
    /// This function is used in a lambda, [[maybe_unused]] is to suppress 'unused' warnings
    [[maybe_unused]] void setCurrentWorld(world::World* world) { currentWorld = world; }

    double getMaxRobotVelocity() const { return maxRobotVelocity; }
    void setMaxRobotVelocity(double maxVelocity) { maxRobotVelocity = maxVelocity; }

    std::string getRoleName() const { return roleName; }
    void setRoleName(std::string name) { roleName = name; }

    AvoidObjects getObjectsToAvoid() const { return avoidObjects; }

    void setShouldAvoidGoalPosts(bool shouldAvoidGoalPosts) {
        if (shouldAvoidGoalPosts != avoidObjects.shouldAvoidGoalPosts) {
            avoidObjects.shouldAvoidGoalPosts = shouldAvoidGoalPosts;
        }
    }

    void setShouldAvoidOutOfField(bool shouldAvoidOutOfField) {
        if (shouldAvoidOutOfField != avoidObjects.shouldAvoidOutOfField) {
            avoidObjects.shouldAvoidOutOfField = shouldAvoidOutOfField;
        }
    }

    void setShouldAvoidOurDefenseArea(bool shouldAvoidOurDefenseArea) {
        if (shouldAvoidOurDefenseArea != avoidObjects.shouldAvoidOurDefenseArea) {
            avoidObjects.shouldAvoidOurDefenseArea = shouldAvoidOurDefenseArea;
        }
    }

    void setShouldAvoidTheirDefenseArea(bool shouldAvoidTheirDefenseArea) {
        if (shouldAvoidTheirDefenseArea != avoidObjects.shouldAvoidTheirDefenseArea) {
            avoidObjects.shouldAvoidTheirDefenseArea = shouldAvoidTheirDefenseArea;
        }
    }

    void setShouldAvoidOurRobots(bool shouldAvoidOurRobots) {
        if (shouldAvoidOurRobots != avoidObjects.shouldAvoidOurRobots) {
            avoidObjects.shouldAvoidOurRobots = shouldAvoidOurRobots;
        }
    }

    void setShouldAvoidTheirRobots(bool shouldAvoidTheirRobots) {
        if (shouldAvoidTheirRobots != avoidObjects.shouldAvoidTheirRobots) {
            avoidObjects.shouldAvoidTheirRobots = shouldAvoidTheirRobots;
        }
    }

    void setShouldAvoidBall(bool shouldAvoidBall) {
        if (shouldAvoidBall != avoidObjects.shouldAvoidBall) {
            avoidObjects.shouldAvoidBall = shouldAvoidBall;
        }
    }

   private:
    /**
     * Current world pointer
     */
    world::World* currentWorld;

    /**
     * Robot this tactic applies to
     */
    std::optional<world::view::RobotView> robot;

    /**
     * Field
     */
    std::optional<Field> field;

    /**
     * View to the ball in the world this tactic executes on
     */
    std::optional<world::view::BallView> ball;

    /**
     * Position to move to
     */
    std::optional<Vector2> positionToMoveTo;

    /**
     * Target velocity to move with
     */
    Vector2 targetVelocity;

    /**
     * Jerk to move with
     */
    double maxJerk = ai::constants::MAX_JERK_DEFAULT;

    /**
     * Position to kick or chip at
     */
    std::optional<Vector2> positionToShootAt;

    /**
     * Position to defend
     */
    std::optional<Vector2> positionToDefend;

    /**
     * Velocity of the kick/chip
     */
    double kickChipVelocity = 0.0;

    /**
     * Type of the kick/chip
     */
    ShotPower shotPower{};

    /**
     * Reference yaw of the robot
     */
    Angle yaw = Angle(0.0);

    /**
     * Whether the dribbler should be on
     */
    bool dribblerOn;

    /**
     * Set the shot to be a kick or chip
     */
    rtt::KickType kickOrChip = rtt::KickType::NO_KICK;

    /**
     * The maximum velocity the robot is allowed to have
     */
    double maxRobotVelocity;

    /**
     * The name of the role associated with this StpInfo
     */
    std::string roleName;

    /**
     * Specify what objects the robot should avoid
     */
    AvoidObjects avoidObjects;
};

/**
 * Util operator<< that allows us to print the status enum
 */
[[maybe_unused]] static std::ostream& operator<<(std::ostream& os, Status status) {
    switch (status) {
        case Status::Waiting:
            return os << "Status::Waiting";
        case Status::Success:
            return os << "Status::Success";
        case Status::Failure:
            return os << "Status::Failure";
        case Status::Running:
            return os << "Status::Running";
        default:
            return os << "INVALID STATUS";
    }
}
}  // namespace rtt::ai::stp

#endif  // RTT_STPINFO_H
