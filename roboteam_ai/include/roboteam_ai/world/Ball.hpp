//
// Created by john on 12/18/19.
//

#ifndef RTT_BALL_HPP
#define RTT_BALL_HPP

#include <proto/WorldBall.pb.h>

#include "roboteam_utils/Vector2.h"

namespace rtt::world {
class World;
}

namespace rtt::world::ball {

/**
 * @brief Ball class that stores info about the ball and adds some utilities
 */
class Ball {
public:
    Vector2 position; /**< Position of the ball */
    Vector2 velocity; /**< Velocity of the ball */
    bool visible = false; /**< Whether the ball is visible by any camera */

    /**
     * @brief Initializes ball at the previously seen position, if the current ball is not visible
     * @param data The current world
     */
    void initBallAtExpectedPosition(const world::World *data) noexcept;

    /**
     * @brief Updates the expected ball end position
     * @param data The current world
     */
    void updateExpectedBallEndPosition(const world::World *data) noexcept;

    /**
     * @brief Places the ball in front of the robot that has the ball, if any
     * @param data The current world
     */
    void updateBallAtRobotPosition(const world::World *data) noexcept;

    /**
     * @brief Create a Ball object with the current data about the ball.
     * @param copy The current data about the ball
     * @param data The current world
     */
    explicit Ball(const proto::WorldBall &copy, const world::World *data);

    /**
     * @brief Default constructor
     */
    Ball() = default;

    /**
     * @brief Default copy assignment operator, does nothing
     */
    Ball &operator=(Ball const &) = default;

    /**
     * @brief Default copy constructor
     */
    Ball(Ball const &) = default;

    /**
     * @brief Default move assignment operator, does nothing
     */
    Ball &operator=(Ball &&) = default;

    /**
     * @brief default move constructor
     */
    Ball(Ball &&) = default;

    /**
     * @brief Default destructor
     */
    ~Ball() = default;
};

}  // namespace rtt::world::ball

#endif  // RTT_BALL_HPP
