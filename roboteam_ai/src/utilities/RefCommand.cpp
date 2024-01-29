#include "utilities/RefCommand.h"

#include <proto/messages_robocup_ssl_referee.pb.h>

#include <string>

namespace rtt {

std::string refCommandToString(RefCommand command) {
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
        case RefCommand::TIMEOUT_US:
            return "TIMEOUT_US";
        case RefCommand::TIMEOUT_THEM:
            return "TIMEOUT_THEM";
        case RefCommand::GOAL_US:
            return "GOAL_US";
        case RefCommand::GOAL_THEM:
            return "GOAL_THEM";
        case RefCommand::BALL_PLACEMENT_US:
            return "BALL_PLACEMENT_US";
        case RefCommand::BALL_PLACEMENT_THEM:
            return "BALL_PLACEMENT_THEM";
        case RefCommand::PRE_HALF:
            return "PRE_HALF";
        case RefCommand::DO_KICKOFF:
            return "DO_KICKOFF";
        case RefCommand::DEFEND_KICKOFF:
            return "DEFEND_KICKOFF";
        case RefCommand::DO_PENALTY:
            return "DO_PENALTY";
        case RefCommand::DEFEND_PENALTY:
            return "DEFEND_PENALTY";
        case RefCommand::PREPARE_SHOOTOUT_US:
            return "PREPARE_SHOOTOUT_US";
        case RefCommand::PREPARE_SHOOTOUT_THEM:
            return "PREPARE_SHOOTOUT_THEM";
        case RefCommand::DO_SHOOTOUT:
            return "DO_SHOOTOUT";
        case RefCommand::DEFEND_SHOOTOUT:
            return "DEFEND_SHOOTOUT";
        case RefCommand::UNDEFINED:
            return "UNDEFINED";
        default:
            return "Warning: unknown RefCommand";
    }
}

std::string protoRefCommandToString(proto::Referee_Command command) {
    switch (command) {
        case proto::Referee_Command_HALT:
            return "HALT";
        case proto::Referee_Command_STOP:
            return "STOP";
        case proto::Referee_Command_NORMAL_START:
            return "NORMAL_START";
        case proto::Referee_Command_FORCE_START:
            return "FORCE_START";
        case proto::Referee_Command_PREPARE_KICKOFF_YELLOW:
            return "PREPARE_KICKOFF_YELLOW";
        case proto::Referee_Command_PREPARE_KICKOFF_BLUE:
            return "PREPARE_KICKOFF_BLUE";
        case proto::Referee_Command_PREPARE_PENALTY_YELLOW:
            return "PREPARE_PENALTY_YELLOW";
        case proto::Referee_Command_PREPARE_PENALTY_BLUE:
            return "PREPARE_PENALTY_BLUE";
        case proto::Referee_Command_DIRECT_FREE_YELLOW:
            return "DIRECT_FREE_YELLOW";
        case proto::Referee_Command_DIRECT_FREE_BLUE:
            return "DIRECT_FREE_BLUE";
        case proto::Referee_Command_TIMEOUT_YELLOW:
            return "TIMEOUT_YELLOW";
        case proto::Referee_Command_TIMEOUT_BLUE:
            return "TIMEOUT_BLUE";
        case proto::Referee_Command_BALL_PLACEMENT_YELLOW:
            return "BALL_PLACEMENT_YELLOW";
        case proto::Referee_Command_BALL_PLACEMENT_BLUE:
            return "BALL_PLACEMENT_BLUE";
        default:
            return "Warning: unknown Referee_Command";
    }
}

std::string protoRefStageToString(proto::Referee_Stage stage) {
    switch (stage) {
        case proto::Referee_Stage_NORMAL_FIRST_HALF_PRE:
            return "NORMAL_FIRST_HALF_PRE";
        case proto::Referee_Stage_NORMAL_FIRST_HALF:
            return "NORMAL_FIRST_HALF";
        case proto::Referee_Stage_NORMAL_HALF_TIME:
            return "NORMAL_HALF_TIME";
        case proto::Referee_Stage_NORMAL_SECOND_HALF_PRE:
            return "NORMAL_SECOND_HALF_PRE";
        case proto::Referee_Stage_NORMAL_SECOND_HALF:
            return "NORMAL_SECOND_HALF";
        case proto::Referee_Stage_EXTRA_TIME_BREAK:
            return "EXTRA_TIME_BREAK";
        case proto::Referee_Stage_EXTRA_FIRST_HALF_PRE:
            return "EXTRA_FIRST_HALF_PRE";
        case proto::Referee_Stage_EXTRA_FIRST_HALF:
            return "EXTRA_FIRST_HALF";
        case proto::Referee_Stage_EXTRA_HALF_TIME:
            return "EXTRA_HALF_TIME";
        case proto::Referee_Stage_EXTRA_SECOND_HALF_PRE:
            return "EXTRA_SECOND_HALF_PRE";
        case proto::Referee_Stage_EXTRA_SECOND_HALF:
            return "EXTRA_SECOND_HALF";
        case proto::Referee_Stage_PENALTY_SHOOTOUT_BREAK:
            return "PENALTY_SHOOTOUT_BREAK";
        case proto::Referee_Stage_PENALTY_SHOOTOUT:
            return "PENALTY_SHOOTOUT";
        case proto::Referee_Stage_POST_GAME:
            return "POST_GAME";
        default:
            return "Warning: unknown Referee_Stage";
    }
}

}  // namespace rtt