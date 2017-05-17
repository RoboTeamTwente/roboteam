#include "roboteam_msgs/RefereeCommand.h"
#include "roboteam_msgs/RefereeData.h"
#include "roboteam_msgs/RefereeStage.h"

#include "roboteam_utils/LastRef.h"
#include "ros/ros.h"

namespace b = boost;

namespace rtt {

std::string refStateToString(RefState s) {
    if (s == RefState::HALT) {
        return "HALT";
    } else if (s == RefState::STOP) {
        return "STOP";
    } else if (s == RefState::NORMAL_START) {
        return "NORMAL_START";
    } else if (s == RefState::FORCED_START) {
        return "FORCED_START";
    } else if (s == RefState::PREPARE_KICKOFF_US) {
        return "PREPARE_KICKOFF_US";
    } else if (s == RefState::PREPARE_KICKOFF_THEM) {
        return "PREPARE_KICKOFF_THEM";
    } else if (s == RefState::PREPARE_PENALTY_US) {
        return "PREPARE_PENALTY_US";
    } else if (s == RefState::PREPARE_PENALTY_THEM) {
        return "PREPARE_PENALTY_THEM";
    } else if (s == RefState::DIRECT_FREE_US) {
        return "DIRECT_FREE_US";
    } else if (s == RefState::DIRECT_FREE_THEM) {
        return "DIRECT_FREE_THEM";
    } else if (s == RefState::INDIRECT_FREE_US) {
        return "INDIRECT_FREE_US";
    } else if (s == RefState::INDIRECT_FREE_THEM) {
        return "INDIRECT_FREE_THEM";
    } else if (s == RefState::TIMEOUT_US) {
        return "TIMEOUT_US";
    } else if (s == RefState::TIMEOUT_THEM) {
        return "TIMEOUT_THEM";
    } else if (s == RefState::GOAL_US) {
        return "GOAL_US";
    } else if (s == RefState::GOAL_THEM) {
        return "GOAL_THEM";
    } else if (s == RefState::BALL_PLACEMENT_US) {
        return "BALL_PLACEMENT_US";
    } else if (s == RefState::BALL_PLACEMENT_THEM) {
        return "BALL_PLACEMENT_THEM";
    // } else if (s == RefState::NORMAL_PLAY) {
        // return "NORMAL_PLAY";
    } else if (s == RefState::DO_KICKOFF) {
        return "DO_KICKOFF";
    } else if (s == RefState::DEFEND_KICKOFF) {
        return "DEFEND_KICKOFF";
    } else if (s == RefState::DO_PENALTY) {
        return "DO_PENALTY";
    } else if (s == RefState::DEFEND_PENALTY) {
        return "DEFEND_PENALTY";
    } else {
        return "UNKNOWN REF STATE";
    }
}

b::optional<RefState> stringToRefState(std::string s) {
    if (s == "HALT") {
        return RefState::HALT;
    } else if (s == "STOP") {
        return RefState::STOP;
    } else if (s == "NORMAL_START") {
        return RefState::NORMAL_START;
    } else if (s == "FORCED_START") {
        return RefState::FORCED_START;
    } else if (s == "PREPARE_KICKOFF_US") {
        return RefState::PREPARE_KICKOFF_US;
    } else if (s == "PREPARE_KICKOFF_THEM") {
        return RefState::PREPARE_KICKOFF_THEM;
    } else if (s == "PREPARE_PENALTY_US") {
        return RefState::PREPARE_PENALTY_US;
    } else if (s == "PREPARE_PENALTY_THEM") {
        return RefState::PREPARE_PENALTY_THEM;
    } else if (s == "DIRECT_FREE_US") {
        return RefState::DIRECT_FREE_US;
    } else if (s == "DIRECT_FREE_THEM") {
        return RefState::DIRECT_FREE_THEM;
    } else if (s == "INDIRECT_FREE_US") {
        return RefState::INDIRECT_FREE_US;
    } else if (s == "INDIRECT_FREE_THEM") {
        return RefState::INDIRECT_FREE_THEM;
    } else if (s == "TIMEOUT_US") {
        return RefState::TIMEOUT_US;
    } else if (s == "TIMEOUT_THEM") {
        return RefState::TIMEOUT_THEM;
    } else if (s == "GOAL_US") {
        return RefState::GOAL_US;
    } else if (s == "GOAL_THEM") {
        return RefState::GOAL_THEM;
    } else if (s == "BALL_PLACEMENT_US") {
        return RefState::BALL_PLACEMENT_US;
    } else if (s == "BALL_PLACEMENT_THEM") {
        return RefState::BALL_PLACEMENT_THEM;
    // } else if (s == "NORMAL_PLAY") {
        // return RefState::NORMAL_PLAY;
    } else if (s == "DO_KICKOFF") {
        return RefState::DO_KICKOFF;
    } else if (s == "DEFEND_KICKOFF") {
        return RefState::DEFEND_KICKOFF;
    } else if (s == "DO_PENALTY") {
        return RefState::DO_PENALTY;
    } else if (s == "DEFEND_PENALTY") {
        return RefState::DEFEND_PENALTY;
    } else {
        return b::none;
    }
}

boost::optional<RefState> toRefState(int refStateInt) {
    if (refStateInt >= 0 && refStateInt <= 17) {
        return static_cast<RefState>(refStateInt);
    }

    return b::none;
}

boost::optional<int> fromRefState(RefState refState) {
    int rawRefState = static_cast<int>(refState);

    if (rawRefState >= 0 && rawRefState < 17) {
        return rawRefState;
    }

    return b::none;
}

roboteam_msgs::RefereeData LastRef::lastRef;
boost::optional<RefState> LastRef::previousRefCommand;
boost::optional<RefState> LastRef::currentRefCommand;

roboteam_msgs::RefereeData LastRef::get() {
    return LastRef::lastRef;
}

RefState LastRef::getState() {
    if (currentRefCommand) {
        return *currentRefCommand;
    } else {
        // Something bad probably happens here
        return static_cast<RefState>(LastRef::lastRef.command.command);
        // Might as well launch a nuclear missile
    }
}

void LastRef::set(roboteam_msgs::RefereeData refCommand) {
    auto previousCurrentRefCommand = currentRefCommand;

    if (auto refStateOpt = toRefState(refCommand.command.command)) {
        currentRefCommand = *refStateOpt;
    } else {
        ROS_ERROR("Received a refstate that is not a refstate! I.e. is more or less than 0 - 17 inclusive. Refstate: %d", refCommand.command.command);
        return;
    }


    if (currentRefCommand != previousCurrentRefCommand) {
        previousRefCommand = previousCurrentRefCommand;
    }

    LastRef::lastRef = refCommand;
}

const std::vector<RefStateTransitionFunction> LastRef::transitions = {
    //TODO, for example:
    [](RefState current, const roboteam_msgs::World& world) {
        return current == RefState::INDIRECT_FREE_THEM && (int) world.us[0].pos.x % 2
                ? boost::optional<RefState>(RefState::NORMAL_START)
                : boost::optional<RefState>();
    }
};

b::optional<RefState> LastRef::getPreviousRefCommand() {
    return previousRefCommand;
}

bool LastRef::hasReceivedFirstCommand() {
    return !!currentRefCommand;
}

}
