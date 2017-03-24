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

/**
 * /enum RefState
 * /brief Used to hold the referee state.
 *
 * A different entity from the RefState used in the ros messages.
 */
enum class RefState {
    HALT = 0,
    STOP,
    NORMAL_START,
    FORCED_START,
    PREPARE_KICKOFF_US,
    PREPARE_KICKOFF_THEM,
    PREPARE_PENALTY_US,
    PREPARE_PENALTY_THEM,
    DIRECT_FREE_US,
    DIRECT_FREE_THEM,
    INDIRECT_FREE_US,
    INDIRECT_FREE_THEM,
    TIMEOUT_US,
    TIMEOUT_THEM,
    GOAL_US,
    GOAL_THEM,
    BALL_PLACEMENT_US,
    BALL_PLACEMENT_THEM,
    NORMAL_PLAY
};

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

    private:
    static roboteam_msgs::RefereeData lastRef;
    static const std::vector<RefStateTransitionFunction> transitions;
};

}
