#pragma once

#include "roboteam_utils/constants.h"
#include <string>

namespace rtt {
    
    typedef unsigned int RobotID;
    
    struct TeamRobot {
        RobotID id;
        bool our_team;
        
        bool operator<(TeamRobot other) const {
            return (id | (our_team << 2)) < (other.id | (other.our_team << 2));
        }
        
        bool operator==(TeamRobot other) const {
            return id == other.id && our_team == other.our_team;
        }
    };

    TeamRobot get_bot_with_color(RobotID id, std::string color);
}
