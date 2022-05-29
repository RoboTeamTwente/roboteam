#pragma once

#include <map>
#include <string>
#include <vector>
#include <optional>

#include <roboteam_utils/Teams.hpp>

namespace rtt::robothub::simulation {
typedef struct SimulationError {
    std::optional<std::string> code;
    std::optional<std::string> message;
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