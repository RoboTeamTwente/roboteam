#pragma once

#include <string>
#include <vector>

namespace rtt::robothub::simulation {
typedef struct SimulationError {
    std::string code;
    std::string message;
} SimulationError;

typedef struct RobotControlFeedback {
    bool isTeamYellow;
    std::vector<int> robotsThatHaveTheBall;
    std::vector<SimulationError> simulationErrors;
} RobotControlFeedback;

typedef struct ConfigurationFeedback {
    std::vector<SimulationError> simulationErrors;
} ConfigurationFeedback;
}  // namespace rtt::robothub::simulation