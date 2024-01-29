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

inline RefCommand sslRefCmdToRefCmd(proto::Referee_Command sslRefCmd, bool isYellow) {
    using RefCommandMap = std::unordered_map<proto::Referee_Command, RefCommand>;

    // static const == they are initialized only once, se we don't allocate on heap every time
    static const RefCommandMap yellowMap = {{proto::Referee_Command_HALT, RefCommand::HALT},
                                            {proto::Referee_Command_STOP, RefCommand::STOP},
                                            {proto::Referee_Command_NORMAL_START, RefCommand::NORMAL_START},
                                            {proto::Referee_Command_FORCE_START, RefCommand::FORCED_START},
                                            {proto::Referee_Command_PREPARE_KICKOFF_YELLOW, RefCommand::PREPARE_KICKOFF_US},
                                            {proto::Referee_Command_PREPARE_KICKOFF_BLUE, RefCommand::PREPARE_KICKOFF_THEM},
                                            {proto::Referee_Command_PREPARE_PENALTY_YELLOW, RefCommand::PREPARE_PENALTY_US},
                                            {proto::Referee_Command_PREPARE_PENALTY_BLUE, RefCommand::PREPARE_PENALTY_THEM},
                                            {proto::Referee_Command_DIRECT_FREE_YELLOW, RefCommand::DIRECT_FREE_US},
                                            {proto::Referee_Command_DIRECT_FREE_BLUE, RefCommand::DIRECT_FREE_THEM},
                                            {proto::Referee_Command_TIMEOUT_YELLOW, RefCommand::TIMEOUT_US},
                                            {proto::Referee_Command_TIMEOUT_BLUE, RefCommand::TIMEOUT_THEM},
                                            {proto::Referee_Command_BALL_PLACEMENT_YELLOW, RefCommand::BALL_PLACEMENT_US},
                                            {proto::Referee_Command_BALL_PLACEMENT_BLUE, RefCommand::BALL_PLACEMENT_THEM}};

    // static const == they are initialized only once, se we don't allocate on heap every time
    static const RefCommandMap blueMap = {{proto::Referee_Command_HALT, RefCommand::HALT},
                                          {proto::Referee_Command_STOP, RefCommand::STOP},
                                          {proto::Referee_Command_NORMAL_START, RefCommand::NORMAL_START},
                                          {proto::Referee_Command_FORCE_START, RefCommand::FORCED_START},
                                          {proto::Referee_Command_PREPARE_KICKOFF_YELLOW, RefCommand::PREPARE_KICKOFF_THEM},
                                          {proto::Referee_Command_PREPARE_KICKOFF_BLUE, RefCommand::PREPARE_KICKOFF_US},
                                          {proto::Referee_Command_PREPARE_PENALTY_YELLOW, RefCommand::PREPARE_PENALTY_THEM},
                                          {proto::Referee_Command_PREPARE_PENALTY_BLUE, RefCommand::PREPARE_PENALTY_US},
                                          {proto::Referee_Command_DIRECT_FREE_YELLOW, RefCommand::DIRECT_FREE_THEM},
                                          {proto::Referee_Command_DIRECT_FREE_BLUE, RefCommand::DIRECT_FREE_US},
                                          {proto::Referee_Command_TIMEOUT_YELLOW, RefCommand::TIMEOUT_THEM},
                                          {proto::Referee_Command_TIMEOUT_BLUE, RefCommand::TIMEOUT_US},
                                          {proto::Referee_Command_BALL_PLACEMENT_YELLOW, RefCommand::BALL_PLACEMENT_THEM},
                                          {proto::Referee_Command_BALL_PLACEMENT_BLUE, RefCommand::BALL_PLACEMENT_US}};

    // .find() != .end() could be replaced by contains in C++20
    if (isYellow && yellowMap.find(sslRefCmd) != yellowMap.end()) {
        return yellowMap.at(sslRefCmd);
    } else if (!isYellow && blueMap.find(sslRefCmd) != yellowMap.end()) {
        return blueMap.at(sslRefCmd);
    }

    RTT_ERROR("Unknown refstate, halting all robots for safety!")
    return RefCommand::HALT;
}

// std::string refCommandToString(RefCommand command);
// std::string protoRefCommandToString(proto::Referee_Command command);
// std::string protoRefStageToString(proto::Referee_Stage stage);

}  // namespace rtt

#endif  // RTT_REFCOMMAND_H
