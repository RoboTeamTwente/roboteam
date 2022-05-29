#pragma once

#include <map>
#include <string>
#include <vector>

#include <roboteam_utils/Teams.hpp>

namespace rtt::robothub::simulation {
typedef struct SimulationError {
    std::string code = "UNKNOWN";
    std::string message = "UNKNOWN";
} SimulationError;

typedef struct RobotControlFeedback {
    rtt::Team color;
    std::map<int, bool> robotIdHasBall;
    std::vector<SimulationError> simulationErrors;
} RobotControlFeedback;

typedef struct ConfigurationFeedback {
    std::vector<SimulationError> simulationErrors;
} ConfigurationFeedback;
}  // namespace rtt::robothub::simulation