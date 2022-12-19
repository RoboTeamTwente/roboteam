#pragma once

#include <string>
#include <vector>

#include <roboteam_utils/Vector2.h>

namespace rtt {

typedef struct RobotPath {
    int robotId = 0;
    std::vector<Vector2> points;

    bool operator==(const RobotPath& other) const;
} RobotPath;

typedef struct RobotSTP {
    int robotId = 0;
    std::string role;
    std::string roleStatus;
    std::string tactic;
    std::string tacticStatus;
    std::string skill;
    std::string skillStatus;

    bool operator==(const RobotSTP& other) const;
} RobotSTP;

typedef struct AIData {
    std::vector<RobotSTP> robotStps;
    std::vector<RobotPath> robotPaths;

    bool operator==(const AIData& other) const;
} AIData;

} // namespace rtt