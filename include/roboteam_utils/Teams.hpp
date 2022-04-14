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

std::string teamToString(Team team) {
    switch (team) {
        case Team::YELLOW:
            return "YELLOW";
        case Team::BLUE:
            return "BLUE";
        default:
            return "UNKNOWN";
    }
}

} // namespace rtt