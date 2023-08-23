#include <Teams.hpp>

namespace rtt {

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

std::string relativeTeamToString(RelativeTeam team) {
    switch (team) {
        case RelativeTeam::US:
            return "US";
        case RelativeTeam::THEM:
            return "THEM";
        default:
            return "UNKNOWN";
    }
}

}  // namespace rtt