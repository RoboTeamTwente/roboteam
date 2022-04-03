#pragma once

#include <vector>
#include <optional>

#include <roboteam_utils/Ball.hpp>
#include <roboteam_utils/Robot.hpp>
#include <roboteam_utils/Field.hpp>

#include <proto/messages_robocup_ssl_referee.pb.h>

namespace rtt {

/* This structure contains the locations of all moving elements from a certain time stamp. */
typedef struct World {
    unsigned long timePoint = 0;            // The timestamp of the world
    unsigned int id = 0;                    // The ID of the world
    std::optional<Ball> ball = std::nullopt;// The ball
    std::vector<Robot> yellowRobots;        // All yellow robots
    std::vector<Robot> blueRobots;          // All blue robots
} World;

/* This represents the complete world (SoonTM also past and guestimated future) */
typedef struct WorldStates {
    World currentWorld;         // The current world
    // World futureWorld;       // TODO: Implement this

    rtt::Field field;           // The field of the world
    proto::SSL_Referee referee; // The latest referee command
} WorldStates;

} // namespace rtt