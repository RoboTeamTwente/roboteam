#pragma once

#include <string>

namespace rtt {

// The colors of the two teams. Yellow is our default
enum class Team {
    YELLOW, BLUE
};

// Teams expressed relatively.
enum class RelativeTeam {
    US, THEM
};

std::string teamToString(Team team);
std::string relativeTeamToString(RelativeTeam team);

} // namespace rtt