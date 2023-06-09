//
// Created by Martin Miksik on 05/05/2023.
//

#ifndef RTT_REFCOMMAND_H
#define RTT_REFCOMMAND_H

#include <unordered_map>

#include <roboteam_utils/Print.h>
#include <proto/messages_robocup_ssl_referee.pb.h>

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
        INDIRECT_FREE_US = 10,
        INDIRECT_FREE_THEM = 11,
        TIMEOUT_US = 12,
        TIMEOUT_THEM = 13,
        GOAL_US = 14,
        GOAL_THEM = 15,
        BALL_PLACEMENT_US = 16,
        BALL_PLACEMENT_THEM = 17,
        PRE_HALF = 26,

        /// CUSTOM GAME STATES ///
        DO_KICKOFF = 18,
        DEFEND_KICKOFF = 19,
        DO_PENALTY = 20,
        DEFEND_PENALTY = 21,
        PREPARE_SHOOTOUT_US = 22,
        PREPARE_SHOOTOUT_THEM = 23,
        DO_SHOOTOUT = 24,
        DEFEND_SHOOTOUT = 25,

        UNDEFINED = -1
    };

    inline RefCommand sslRefCmdToRefCmd(proto::SSL_Referee_Command sslRefCmd, bool isYellow) {
        using RefCommandMap = std::unordered_map<proto::SSL_Referee_Command, RefCommand>;

        // static const == they are initialized only once, se we don't allocate on heap every time
        static const RefCommandMap yellowMap = {
            {proto::SSL_Referee_Command_HALT, RefCommand::HALT},
            {proto::SSL_Referee_Command_STOP, RefCommand::STOP},
            {proto::SSL_Referee_Command_NORMAL_START, RefCommand::NORMAL_START},
            {proto::SSL_Referee_Command_FORCE_START, RefCommand::FORCED_START},
            {proto::SSL_Referee_Command_PREPARE_KICKOFF_YELLOW, RefCommand::PREPARE_KICKOFF_US},
            {proto::SSL_Referee_Command_PREPARE_KICKOFF_BLUE, RefCommand::PREPARE_KICKOFF_THEM},
            {proto::SSL_Referee_Command_PREPARE_PENALTY_YELLOW, RefCommand::PREPARE_PENALTY_US},
            {proto::SSL_Referee_Command_PREPARE_PENALTY_BLUE, RefCommand::PREPARE_SHOOTOUT_THEM},
            {proto::SSL_Referee_Command_DIRECT_FREE_YELLOW, RefCommand::DIRECT_FREE_US},
            {proto::SSL_Referee_Command_DIRECT_FREE_BLUE, RefCommand::DIRECT_FREE_THEM},
            {proto::SSL_Referee_Command_INDIRECT_FREE_YELLOW, RefCommand::INDIRECT_FREE_US},
            {proto::SSL_Referee_Command_INDIRECT_FREE_BLUE, RefCommand::INDIRECT_FREE_THEM},
            {proto::SSL_Referee_Command_TIMEOUT_YELLOW, RefCommand::TIMEOUT_US},
            {proto::SSL_Referee_Command_TIMEOUT_BLUE, RefCommand::TIMEOUT_THEM},
            {proto::SSL_Referee_Command_BALL_PLACEMENT_YELLOW, RefCommand::BALL_PLACEMENT_US},
            {proto::SSL_Referee_Command_BALL_PLACEMENT_BLUE, RefCommand::BALL_PLACEMENT_THEM}
        };

        // static const == they are initialized only once, se we don't allocate on heap every time
        static const RefCommandMap blueMap = {
            {proto::SSL_Referee_Command_HALT, RefCommand::HALT},
            {proto::SSL_Referee_Command_STOP, RefCommand::STOP},
            {proto::SSL_Referee_Command_NORMAL_START, RefCommand::NORMAL_START},
            {proto::SSL_Referee_Command_FORCE_START, RefCommand::FORCED_START},
            {proto::SSL_Referee_Command_PREPARE_KICKOFF_YELLOW, RefCommand::PREPARE_KICKOFF_THEM},
            {proto::SSL_Referee_Command_PREPARE_KICKOFF_BLUE, RefCommand::PREPARE_KICKOFF_US},
            {proto::SSL_Referee_Command_PREPARE_PENALTY_YELLOW, RefCommand::PREPARE_PENALTY_THEM},
            {proto::SSL_Referee_Command_PREPARE_PENALTY_BLUE, RefCommand::PREPARE_SHOOTOUT_US},
            {proto::SSL_Referee_Command_DIRECT_FREE_YELLOW, RefCommand::DIRECT_FREE_THEM},
            {proto::SSL_Referee_Command_DIRECT_FREE_BLUE, RefCommand::DIRECT_FREE_US},
            {proto::SSL_Referee_Command_INDIRECT_FREE_YELLOW, RefCommand::INDIRECT_FREE_THEM},
            {proto::SSL_Referee_Command_INDIRECT_FREE_BLUE, RefCommand::INDIRECT_FREE_US},
            {proto::SSL_Referee_Command_TIMEOUT_YELLOW, RefCommand::TIMEOUT_THEM},
            {proto::SSL_Referee_Command_TIMEOUT_BLUE, RefCommand::TIMEOUT_US},
            {proto::SSL_Referee_Command_BALL_PLACEMENT_YELLOW, RefCommand::BALL_PLACEMENT_THEM},
            {proto::SSL_Referee_Command_BALL_PLACEMENT_BLUE, RefCommand::BALL_PLACEMENT_US}
        };

        // .find() != .end() could be replaced by contains in C++20
        if (isYellow && yellowMap.find(sslRefCmd) != yellowMap.end()) {
            return yellowMap.at(sslRefCmd);
        } else if (!isYellow && blueMap.find(sslRefCmd) != yellowMap.end()){
            return blueMap.at(sslRefCmd);
        }

        RTT_ERROR("Unknown refstate, halting all robots for safety!")
        return RefCommand::HALT;
    }

    std::string refCommandToString(RefCommand command);
    std::string protoRefCommandToString(proto::SSL_Referee_Command command);
    std::string protoRefStageToString(proto::SSL_Referee_Stage stage);

}  // namespace rtt

#endif  // RTT_REFCOMMAND_H
