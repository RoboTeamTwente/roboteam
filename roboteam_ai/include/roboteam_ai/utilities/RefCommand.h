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
    BALL_PLACEMENT_US_DIRECT = 22,  // Ball placement before a direct free kick
    DIRECT_FREE_US_STOP = 23,       // Direct free kick us directly after a stop
    DIRECT_FREE_THEM_STOP = 24,     // Direct free kick them directly after a stop

    UNDEFINED = -1
};

/**
 * @brief Overload the << operator for the RefCommand enum
 * @param os The output stream
 * @param command The RefCommand to print
 * @return The output stream
 */

inline std::ostream &operator<<(std::ostream &os, const RefCommand &command) {
    switch (command) {
        case RefCommand::HALT:
            os << "HALT";
            break;
        case RefCommand::STOP:
            os << "STOP";
            break;
        case RefCommand::NORMAL_START:
            os << "NORMAL_START";
            break;
        case RefCommand::FORCED_START:
            os << "FORCED_START";
            break;
        case RefCommand::PREPARE_KICKOFF_US:
            os << "PREPARE_KICKOFF_US";
            break;
        case RefCommand::PREPARE_KICKOFF_THEM:
            os << "PREPARE_KICKOFF_THEM";
            break;
        case RefCommand::PREPARE_PENALTY_US:
            os << "PREPARE_PENALTY_US";
            break;
        case RefCommand::PREPARE_PENALTY_THEM:
            os << "PREPARE_PENALTY_THEM";
            break;
        case RefCommand::DIRECT_FREE_US:
            os << "DIRECT_FREE_US";
            break;
        case RefCommand::DIRECT_FREE_THEM:
            os << "DIRECT_FREE_THEM";
            break;
        case RefCommand::DIRECT_FREE_US_STOP:
            os << "DIRECT_FREE_US_STOP";
            break;
        case RefCommand::DIRECT_FREE_THEM_STOP:
            os << "DIRECT_FREE_THEM_STOP";
            break;
        case RefCommand::TIMEOUT_US:
            os << "TIMEOUT_US";
            break;
        case RefCommand::TIMEOUT_THEM:
            os << "TIMEOUT_THEM";
            break;
        case RefCommand::BALL_PLACEMENT_US:
            os << "BALL_PLACEMENT_US";
            break;
        case RefCommand::BALL_PLACEMENT_US_DIRECT:
            os << "BALL_PLACEMENT_US_DIRECT";
            break;
        case RefCommand::BALL_PLACEMENT_THEM:
            os << "BALL_PLACEMENT_THEM";
            break;
        case RefCommand::KICKOFF_US:
            os << "KICKOFF_US";
            break;
        case RefCommand::KICKOFF_THEM:
            os << "KICKOFF_THEM";
            break;
        case RefCommand::PENALTY_US:
            os << "PENALTY_US";
            break;
        case RefCommand::PENALTY_THEM:
            os << "PENALTY_THEM";
            break;
        case RefCommand::UNDEFINED:
            os << "UNDEFINED";
            break;
    }
    return os;
}
}  // namespace rtt

#endif  // RTT_REFCOMMAND_H
