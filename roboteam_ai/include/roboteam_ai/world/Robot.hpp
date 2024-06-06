#ifndef RTT_ROBOT_HPP
#define RTT_ROBOT_HPP

#include <proto/RobotFeedback.pb.h>
#include <proto/WorldRobot.pb.h>

#include <optional>

#include "Team.hpp"
#include "roboteam_utils/Angle.h"
#include "world/views/BallView.hpp"

namespace rtt::world::robot {

/**
 * @brief Structure that stores info about whether the robot has the ball
 */
struct hasBallInfo {
    bool hasBall; /**< Indicates whether the robot has the ball */
    int score;    /**< Gives a score to the confidence of this info */
};

/**
 * @brief Class that defines the Robot. The robot class stores data of everything that has to do with a robot.
 * It also has some utilities for determining information about the robot.
 */
class Robot {
   private:
    int id;    /**< ID of the robot */
    Team team; /**< Team the robot belongs to */

    Vector2 pos; /**< Position of the robot */
    Vector2 vel; /**< Velocity of the robot */
    Angle yaw;   /**< Yaw of the robot */

    double distanceToBall;  /**< Distance from the robot to the ball */
    double angleDiffToBall; /**< Angle of the robot relative to the ball */

    double angularVelocity; /**< Angular velocity of the ball */
    bool batteryLow{false}; /**< Indicates whether the battery of the robot is low */

    bool workingDribbler;     /**< Indicates whether the robot has a working dribbler */
    bool workingBallSensor{}; /**< Indicates whether the robot has a working ball sensor */

    bool ballSensorSeesBall{}; /**< Indicates whether the ball sensor sees the ball */
    bool dribblerSeesBall{};   /**< Indicates whether the dribbler sees the ball */
    bool robotHasBall{};       /**< Indicates whether the robot has the ball */

    static inline std::unordered_map<int, hasBallInfo> hasBallUpdateMap; /**< Map that stores a score that indicates how likely we think it is that each robot has the ball */

   private:
    /**
     * @brief Updates the robot information according to the feedback
     * @param feedback The processed feedback received from the robot
     */
    void updateFromFeedback(const proto::RobotProcessedFeedback &feedback) noexcept;

    /**
     * @brief Determines which robot has the ball
     * @param ball The current ball data
     */
    void updateHasBallMap(std::optional<view::BallView> &ball);

    /**
     * @brief Sets the robot's yaw to the specified yaw
     * @param yaw The desired yaw for the robot
     */
    void setYaw(const Angle &yaw) noexcept;

    /**
     * @brief Set the batteryLow boolean
     * @param batteryLow Indicates whether the battery of the robot is low
     */
    void setBatteryLow(bool batteryLow) noexcept;

    /**
     * @brief Set the workingBallSensor boolean
     * @param workingBallSensor Indicates whether the ball sensor of this robot is working
     */
    void setWorkingBallSensor(bool workingBallSensor) noexcept;

    /**
     * @brief Set the distance for the robot to the ball
     * @param distanceToBall The distance from the robot to the ball
     */
    void setDistanceToBall(double distanceToBall) noexcept;

    /**
     * @brief Set the yaw difference to the ball
     * @param _angleDiffToBall The Angle of the robot relative to the ball
     */
    void setAngleDiffToBall(double _angleDiffToBall) noexcept;

    /**
     * @brief Set the ballSensorSeesBall boolean
     * @param _seesBall Indicates whether the ball sensor of the robot sees the ball
     */
    void setBallSensorSeesBall(bool _seesBall) noexcept;

    /**
     * @brief Set the dribblerSeesBall boolean
     * @param _seesBall Indicates whether the dribbler of the robot sees the ball
     */
    void setDribblerSeesBall(bool _seesBall) noexcept;

    /**
     * @brief Set the hasBall boolean
     * @param _hasBall Indicates whether the robot has the ball
     */
    void setHasBall(bool _hasBall) noexcept;

   public:
    /**
     * @brief Get the ID of the robot
     * @return The ID of the robot
     */
    [[nodiscard]] int getId() const noexcept;

    /**
     * @brief Get the Team that the robot belongs to
     * @return The team that the robot belongs to
     */
    [[nodiscard]] Team getTeam() const noexcept;

    /**
     * @brief Get the position of the robot
     * @return The position of the robot
     */
    [[nodiscard]] const Vector2 &getPos() const noexcept;

    /**
     * @brief Get the velocity of the robot
     * @return The velocity of the robot
     */
    [[nodiscard]] const Vector2 &getVel() const noexcept;

    /**
     * @brief Get the yaw of the robot
     * @return The yaw of the robot
     */
    [[nodiscard]] const Angle &getYaw() const noexcept;

    /**
     * @brief Get the angular velocity of the robot
     * @return The angular velocity of the robot
     */
    [[nodiscard]] double getAngularVelocity() const noexcept;

    /**
     * @brief Checks whether the battery of the robot is low
     * @return A boolean that indicates whether the battery of the robot is low
     */
    [[nodiscard]] bool isBatteryLow() const noexcept;

    /**
     * @brief Checks whether the dribbler of the robot is working
     * @return A boolean that indicates whether the dribbler of the robot is working
     */
    [[nodiscard]] bool isWorkingDribbler() const noexcept;

    /**
     * @brife Checks whether the ball sensor of the robot is working
     * @return A boolean that indicates whether the ball sensor of the robot is working
     */
    [[nodiscard]] bool isWorkingBallSensor() const noexcept;

    /**
     * @brief Checks whether the robot has the ball
     * @return A boolean that indicates whether the robot has the ball
     */
    [[nodiscard]] bool hasBall() const noexcept;

    /**
     * @brief Get the distance from the robot to the ball
     * @return The distance from the robot to the ball
     */
    [[nodiscard]] double getDistanceToBall() const noexcept;

    /**
     * @brief Get the yaw of the robot relative to the ball
     * @return The yaw of the robot relative to the ball
     */
    [[nodiscard]] double getAngleDiffToBall() const noexcept;

   public:
    /**
     * @brief Explicit constructor of the Robot class
     * @param copy The observed robot that this robot is based on
     * @param team The team that this robot belongs to
     * @param ball Data about the ball
     */
    explicit Robot(const proto::WorldRobot &copy, Team team = both, std::optional<rtt::world::view::BallView> ball = std::nullopt);

    /**
     * @brief Default copy assignment operator
     */
    Robot &operator=(Robot const &) = default;

    /**
     * @brief Default copy constructor of the Robot class
     */
    Robot(Robot const &) = default;

    /**
     * @brief Default move assignment operator
     */
    Robot &operator=(Robot &&) = default;

    /**
     * @brief Default move constructor of the Robot class
     */
    Robot(Robot &&) = default;
};
}  // namespace rtt::world::robot

#endif  // RTT_ROBOT_HPP
