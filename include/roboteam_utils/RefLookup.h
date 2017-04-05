#pragma once

#include <map>
#include <string>
#include <boost/optional/optional_fwd.hpp>

namespace rtt {

extern const std::map<std::string,int> refstagelookup;
extern const std::map <std::string,int> refcommandlookup;
extern const std::map <int, std::string> refcommandlookdown;

boost::optional<std::string> getRefCommandName(int cmd);

}
