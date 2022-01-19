#pragma once

#include <utilities.h>

#include <map>
#include <string>
#include <vector>

namespace rtt::robothub::simulation {
typedef struct SimulationError {
    std::string code;
    std::string message;
} SimulationError;

typedef struct RobotControlFeedback {
    utils::TeamColor color;
    std::map<int, bool> robotIdHasBall;
    std::vector<SimulationError> simulationErrors;
} RobotControlFeedback;

typedef struct ConfigurationFeedback {
    std::vector<SimulationError> simulationErrors;
} ConfigurationFeedback;
}  // namespace rtt::robothub::simulation