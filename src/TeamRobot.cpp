#include "roboteam_utils/TeamRobot.h"

namespace rtt {

TeamRobot get_bot_with_color(RobotID id, std::string color) {
    std::string our_color;
    get_PARAM_OUR_COLOR(our_color);
    return {id, our_color == color};
}

}
