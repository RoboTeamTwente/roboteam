#pragma once

#include "roboteam_msgs/RefereeCommand.h"
#include "roboteam_msgs/RefereeData.h"
#include "roboteam_msgs/RefereeStage.h"
#include "roboteam_msgs/World.h"
#include "LastWorld.h"
#include "boost/optional.hpp"
#include <functional>
#include <vector>

namespace rtt {

// Doen:
// refcommandlookups met enum en niet met ints
// refstateswitch en strategycomposer schrijven met enums
// use the enum everywhere, conversion stuff should happen in lastref
//
// Check:
// Refstates expliciet nummeren
// Refstate eigen states toevoegen

/**
 * /enum RefState
 * /brief Used to hold the referee state.
 *
 * A different entity from the RefState used in the ros messages.
 */
enum class RefState {
    // Ref states as dictated by RoboCup SSL
    // (with corresponding numeral identifiers - feel free to convert to and from)
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

    // Custom extended refstates
    // (notice they have no numeral identifiers - don't use them like that)
    NORMAL_PLAY,
    DO_KICKOFF,
    DEFEND_KICKOFF,
    DO_PENALTY,
    DEFEND_PENALTY,
    DO_INDIRECT,
    DEFEND_INDIRECT,
    DO_DIRECT,
    DEFEND_DIRECT
} ;

boost::optional<RefState> toRefState(int refStateInt);
boost::optional<int> fromRefState(RefState refState);

constexpr size_t REF_STATE_COUNT = static_cast<size_t>(RefState::NORMAL_PLAY) + 1;

typedef std::function<boost::optional<RefState>(RefState, const roboteam_msgs::World&)> RefStateTransitionFunction;

/**
 * \class LastRef
 * \brief Utility class to keep the last received referee state.
 */
class LastRef {
    public:
    static roboteam_msgs::RefereeData get();
    /**
     * \brief Sets the refstate.
     *        Only to be used when a new refstate has been received.
     */
    static void set(roboteam_msgs::RefereeData refCommand);
    
    static RefState getState();
    static int getPreviousRefCommand();

    private:
    static roboteam_msgs::RefereeData lastRef;
    static int previousRefCommand;
    static const std::vector<RefStateTransitionFunction> transitions;
};

}
