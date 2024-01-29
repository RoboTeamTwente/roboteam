//
// Created by Martin Miksik on 05/05/2023.
//

#ifndef RTT_REFCOMMAND_H
#define RTT_REFCOMMAND_H

#include <proto/messages_robocup_ssl_referee.pb.h>
#include <roboteam_utils/Print.h>

#include <unordered_map>

namespace rtt {
/**
 * @brief Enumerators for the different game state given by the referee
 */
enum class RefCommand {
    /// OFFICIAL REFEREE GAME STATES ///
    HALT = 0,
    STOP = 1,
    NORMAL_START = 2,
    FORCED_START = 3,
    PREPARE_KICKOFF_US = 4,
    PREPARE_KICKOFF_THEM = 5,
    PREPARE_PENALTY_US = 6,
    PREPARE_PENALTY_THEM = 7,
    DIRECT_FREE_US = 8,
    DIRECT_FREE_THEM = 9,
    TIMEOUT_US = 12,
    TIMEOUT_THEM = 13,
    BALL_PLACEMENT_US = 16,
    BALL_PLACEMENT_THEM = 17,

    /// CUSTOM GAME STATES ///
    KICKOFF_US = 18,
    KICKOFF_THEM = 19,
    PENALTY_US = 20,
    PENALTY_THEM = 21,

    UNDEFINED = -1
};
}  // namespace rtt

#endif  // RTT_REFCOMMAND_H
