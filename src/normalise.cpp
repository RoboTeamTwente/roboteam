#include "roboteam_utils/normalise.h"

namespace rtt {

roboteam_msgs::RobotCommand rotateRobotCommand(roboteam_msgs::RobotCommand const & command) {
    roboteam_msgs::RobotCommand newCommand;

    newCommand.id             = command.id;
    newCommand.active         = command.active;
    newCommand.w              = command.w;
    newCommand.dribbler       = command.dribbler;
    newCommand.kicker         = command.kicker;
    newCommand.kicker_forced  = command.kicker_forced;
    newCommand.kicker_vel     = command.kicker_vel;
    newCommand.chipper        = command.chipper;
    newCommand.chipper_forced = command.chipper_forced;
    newCommand.chipper_vel    = command.chipper_vel;

    newCommand.x_vel = -command.x_vel;
    newCommand.y_vel = -command.y_vel;
    
    return newCommand;
}

} // rtt
