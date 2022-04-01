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
    unsigned long timePoint;
    unsigned int id;
    std::optional<Ball> ball;
    std::vector<Robot> yellowRobots;
    std::vector<Robot> blueRobots;
} World;

/* This represents the complete world (SoonTM also past and guestimated future) */
typedef struct WorldStates {
    World currentWorld;
    // World futureWorld;  // TODO: Implement this

    rtt::Field field;
    proto::SSL_Referee referee;
} WorldStates;

} // namespace rtt