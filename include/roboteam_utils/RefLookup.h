#pragma once

#include <map>
#include <set>
#include <string>
#include <boost/optional/optional_fwd.hpp>

namespace rtt {

extern const std::map<std::string,int> refstagelookup;
extern const std::map<std::string,int> refcommandlookup;
extern const std::map<int, std::string> refcommandlookdown;
extern const std::set<int> implicitNormalStartRefCommands;

bool isImplicitNormalStartCommand(int cmd);

boost::optional<std::string> getRefCommandName(int cmd);

}
