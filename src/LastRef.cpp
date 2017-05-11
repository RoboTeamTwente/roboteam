#include "roboteam_msgs/RefereeCommand.h"
#include "roboteam_msgs/RefereeData.h"
#include "roboteam_msgs/RefereeStage.h"

#include "roboteam_utils/LastRef.h"
#include "ros/ros.h"

namespace b = boost;

namespace rtt {

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
int LastRef::previousRefCommand = -1;

roboteam_msgs::RefereeData LastRef::get() {
    return LastRef::lastRef;
}

RefState LastRef::getState() {
    return static_cast<RefState>(LastRef::lastRef.command.command);
}

void LastRef::set(roboteam_msgs::RefereeData refCommand) {
    if (refCommand.command.command != lastRef.command.command) {
        previousRefCommand = lastRef.command.command;
    }

    LastRef::lastRef = refCommand;
}

const std::vector<RefStateTransitionFunction> LastRef::transitions = {
    //TODO, for example:
    [](RefState current, const roboteam_msgs::World& world) {
        return current == RefState::INDIRECT_FREE_THEM && (int) world.us[0].pos.x % 2
                ? boost::optional<RefState>(RefState::NORMAL_PLAY)
                : boost::optional<RefState>();
    }
};

int LastRef::getPreviousRefCommand() {
    return previousRefCommand;
}

}
