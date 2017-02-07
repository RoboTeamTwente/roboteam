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
    };

    inline TeamRobot get_bot_with_color(RobotID id, std::string color) {
        std::string our_color;
        get_PARAM_OUR_COLOR(our_color);
        return {id, our_color == color};
    }
}