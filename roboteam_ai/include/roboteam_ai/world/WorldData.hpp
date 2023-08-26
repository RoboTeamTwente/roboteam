//
// Created by john on 12/16/19.
//

#ifndef RTT_WORLD_DATA_HPP
#define RTT_WORLD_DATA_HPP

#include <proto/GameSettings.pb.h>
#include <proto/RobotFeedback.pb.h>
#include <proto/World.pb.h>

#include <optional>
#include <vector>

#include "utilities/GameSettings.h"
#include "world/views/RobotView.hpp"

namespace rtt::world {
class World;

namespace robot {
class Robot;
}  // namespace robot

/**
 * @brief WorldData class that holds data about a specific time point in the world.
 * Holds:
 *  Robots
 *  Ball
 *  Time point
 */
class WorldData {
    friend class World;

   private:
    /**
     * @brief Constructs new world data
     * @param protoMsg Proto message to construct he world from
     * @param feedback Feedback to apply to robots that'll be constructed
     * Ownership is taken of protoMsg
     * //TODO: This is documentation is outdated, where is feedback, and data doc is missing
     */
    WorldData(const World *data, proto::World &protoMsg) noexcept;

    std::vector<rtt::world::robot::Robot> robots;             /**< Owning container of robots */
    std::vector<world::view::RobotView> robotsNonOwning = {}; /**< Non owning vector of views */
    std::vector<world::view::RobotView> us = {};              /**< Non-owning container of Robot const* const's (aka RobotView) for our team */
    std::vector<world::view::RobotView> them = {};            /** Non-owning container of RobotViews of the enemy team< */
    std::optional<rtt::world::ball::Ball> ball;               /**< Optional ball, None variant if not visible */
    uint64_t time{};                                          /**<Timestamp identical to the protobuf message's time() */

   public:
    /**
     * @brief Default constructor for default STL container initialization
     */
    WorldData() = default;

    /**
     * @brief Copy assignment operator and constructor
     * Copy over members and set the view vectors
     */
    WorldData &operator=(WorldData const &) noexcept;

    /**
     * @brief Copy constructor for the WorldData class
     */
    WorldData(WorldData const &) noexcept;

    /**
     * @brief Move constructor, simply moves all members
     * @param old Data to move
     */
    WorldData(WorldData &&old) = default;

    /**
     * @brief Move assignment operator
     * @return *this
     */
    WorldData &operator=(WorldData &&) = default;

    /**
     * @brief Does exactly what it says, it sets the internal vectors for the views
     */
    void setViewVectors() noexcept;

    /**
     * @brief Gets a non-owning container of robots that are in our team
     * @return this->us
     */
    [[nodiscard]] std::vector<view::RobotView> const &getUs() const noexcept;

    /**
     * @brief Gets a non-owning container of robots that are in the enemy team
     * @return this->them
     */
    [[nodiscard]] std::vector<view::RobotView> const &getThem() const noexcept;

    /**
     * @brief Gets a Some or None non-owning variant of a Ball, aka a BallView
     * @return ball ? Some(BallView) : None
     */
    [[nodiscard]] std::optional<view::BallView> getBall() const noexcept;

    /**
     * @brief Gets the internal vector of robot views
     * @return A const& to it
     */
    [[nodiscard]] const std::vector<view::RobotView> &getRobotsNonOwning() const noexcept;

    /**
     * @brief Gets the time for the current contained world
     * @return The time from the current world
     */
    [[nodiscard]] uint64_t getTime() const noexcept;
};
}  // namespace rtt::world

#endif  // RTT_WORLD_DATA_HPP
