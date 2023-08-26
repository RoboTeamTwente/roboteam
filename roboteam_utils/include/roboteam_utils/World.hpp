#pragma once

#include <optional>
#include <roboteam_utils/Ball.hpp>
#include <roboteam_utils/Field.hpp>
#include <roboteam_utils/Robot.hpp>
#include <vector>

namespace rtt {

/* This structure contains the locations of all moving elements from a certain time stamp. */
typedef struct World {
    unsigned long timePoint = 0;      // The timestamp of the world
    unsigned int id = 0;              // The ID of the world
    std::optional<Ball> ball;         // The ball
    std::vector<Robot> yellowRobots;  // All yellow robots
    std::vector<Robot> blueRobots;    // All blue robots

    bool operator==(const World& other) const;
} World;

/* This represents the complete world (SoonTM also guestimated future) */
typedef struct WorldStates {
    World currentWorld;  // The current world
    // World futureWorld;       // TODO: Implement this

    rtt::Field field;  // The field of the world

    bool operator==(const WorldStates& other) const;
} WorldStates;

}  // namespace rtt