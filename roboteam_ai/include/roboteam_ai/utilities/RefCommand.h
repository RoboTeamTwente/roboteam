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
    PREPARE_FORCED_START = 25,      // The state before a forced start, after ball placement them

    UNDEFINED = -1
};

/**
 * @brief Convert a RefCommand into a string
 * @param command The RefCommand to print
 * @return A string of the RefCommand
 */

inline std::string getNameOfRefCommand(const RefCommand &command) {
    switch (command) {
        case RefCommand::HALT:
            return "HALT";
        case RefCommand::STOP:
            return "STOP";
        case RefCommand::NORMAL_START:
            return "NORMAL_START";
        case RefCommand::FORCED_START:
            return "FORCED_START";
        case RefCommand::PREPARE_KICKOFF_US:
            return "PREPARE_KICKOFF_US";
        case RefCommand::PREPARE_KICKOFF_THEM:
            return "PREPARE_KICKOFF_THEM";
        case RefCommand::PREPARE_PENALTY_US:
            return "PREPARE_PENALTY_US";
        case RefCommand::PREPARE_PENALTY_THEM:
            return "PREPARE_PENALTY_THEM";
        case RefCommand::DIRECT_FREE_US:
            return "DIRECT_FREE_US";
        case RefCommand::DIRECT_FREE_THEM:
            return "DIRECT_FREE_THEM";
        case RefCommand::DIRECT_FREE_US_STOP:
            return "DIRECT_FREE_US_STOP";
        case RefCommand::DIRECT_FREE_THEM_STOP:
            return "DIRECT_FREE_THEM_STOP";
        case RefCommand::TIMEOUT_US:
            return "TIMEOUT_US";
        case RefCommand::TIMEOUT_THEM:
            return "TIMEOUT_THEM";
        case RefCommand::BALL_PLACEMENT_US:
            return "BALL_PLACEMENT_US";
        case RefCommand::BALL_PLACEMENT_US_DIRECT:
            return "BALL_PLACEMENT_US_DIRECT";
        case RefCommand::BALL_PLACEMENT_THEM:
            return "BALL_PLACEMENT_THEM";
        case RefCommand::KICKOFF_US:
            return "KICKOFF_US";
        case RefCommand::KICKOFF_THEM:
            return "KICKOFF_THEM";
        case RefCommand::PENALTY_US:
            return "PENALTY_US";
        case RefCommand::PENALTY_THEM:
            return "PENALTY_THEM";
        case RefCommand::PREPARE_FORCED_START:
            return "PREPARE_FORCED_START";
        case RefCommand::UNDEFINED:
            return "UNDEFINED";
        default:
            return "WARNING";
    }
};

/**
 * @brief Overload the << operator for the RefCommand enum
 * @param os The output stream
 * @param command The RefCommand to print
 * @return The output stream
 */

inline std::ostream &operator<<(std::ostream &os, const RefCommand &command) {
    os << getNameOfRefCommand(command);
    return os;
}
}  // namespace rtt

#endif  // RTT_REFCOMMAND_H
