#pragma once

#include <map>
#include <set>
#include <string>
#include <boost/optional/optional_fwd.hpp>

#include "roboteam_utils/LastRef.h"

namespace rtt {

extern const std::map<std::string,int> refstagelookup;
// extern const std::map<std::string,int> refcommandlookup;
// extern const std::map<int, std::string> refcommandlookdown;
// extern const std::set<int> implicitNormalStartRefCommands;

bool isTwoState(boost::optional<RefState> previousCmd, RefState currentCmd);
boost::optional<RefState> getFirstState(boost::optional<RefState> previousCmd, RefState currentCmd);
boost::optional<RefState> getFirstState();
RefState getExtendedState();

// boost::optional<std::string> getRefCommandName(int cmd);

}
