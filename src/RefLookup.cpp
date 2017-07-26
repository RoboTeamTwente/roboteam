#include <map>
#include <set>
#include <boost/optional.hpp>

#include "roboteam_utils/RefLookup.h"
#include "roboteam_utils/LastRef.h"
#include "roboteam_msgs/RefereeCommand.h"

namespace b = boost;

namespace rtt {

const std::map <std::string,int> refstagelookup = {
	{"NORMAL_FIRST_HALF_PRE", 0},
	{"NORMAL_FIRST_HALF", 1},
	{"NORMAL_HALF_TIME", 2},
	{"NORMAL_SECOND_HALF_PRE", 3},
	{"NORMAL_SECOND_HALF", 4},
	{"EXTRA_TIME_BREAK", 5},
	{"EXTRA_FIRST_HALF_PRE", 6},
	{"EXTRA_FIRST_HALF", 7},
	{"EXTRA_HALF_TIME", 8},
	{"EXTRA_SECOND_HALF_PRE", 9},
	{"EXTRA_SECOND_HALF", 10},
	{"PENALTY_SHOOTOUT_BREAK", 11},
	{"PENALTY_SHOOTOUT", 12},
	{"POST_GAME", 13}
};

namespace b = boost;

const std::map<std::pair<boost::optional<RefState>, RefState>, RefState> twoStatePairs = {
    { {RefState::PREPARE_KICKOFF_US, RefState::NORMAL_START}, 
        RefState::DO_KICKOFF 
    }, 
    { {RefState::PREPARE_KICKOFF_THEM, RefState::NORMAL_START},
        RefState::DEFEND_KICKOFF 
    },
    { {RefState::PREPARE_PENALTY_US, RefState::NORMAL_START},
        RefState::DO_PENALTY 
    },
    { {RefState::PREPARE_PENALTY_THEM, RefState::NORMAL_START},
        RefState::DEFEND_PENALTY 
    },
    { {b::none, RefState::INDIRECT_FREE_US},
        RefState::INDIRECT_FREE_US
    },
    { {b::none, RefState::INDIRECT_FREE_THEM},
        RefState::INDIRECT_FREE_THEM
    },
    { {b::none, RefState::DIRECT_FREE_US},
        RefState::DIRECT_FREE_US
    },
    { {b::none, RefState::DIRECT_FREE_THEM},
        RefState::DIRECT_FREE_THEM
    },
} ;

bool isTwoState(b::optional<RefState> previousCmdOpt, RefState currentCmd) {
    return twoStatePairs.find({previousCmdOpt, currentCmd}) != twoStatePairs.end()
        || twoStatePairs.find({b::none, currentCmd}) != twoStatePairs.end()
        ;
}

/**
 *  Returns the first stage from the two stage pair previousCmd and currentCmd.
 *  The second stage is always normal play.
 */
b::optional<RefState> getFirstState(boost::optional<RefState> previousCmdOpt, RefState currentCmd) {
    auto stateIt = twoStatePairs.find({previousCmdOpt, currentCmd});
    if (stateIt != twoStatePairs.end()) {
        return stateIt->second;
    } else {
        stateIt = twoStatePairs.find({b::none, currentCmd});
        if (stateIt != twoStatePairs.end()) {
            return stateIt->second;
        }
    }

    return b::none;
}

b::optional<RefState> getFirstState() {
    return getFirstState( LastRef::getPreviousRefCommand()
                        , LastRef::getState()
                        );
}

RefState getExtendedState() {
    if (auto possibleFirstState = getFirstState()) {
        return *possibleFirstState;
    } else {
        return LastRef::getState();
    }
}

}
