#include "roboteam_utils/normalise.h"
#include "roboteam_utils/constants.h"

#include <string>


namespace rtt {

using namespace roboteam_msgs;

World normalize_world(World world) {
    bool should_normalize = false;
    std::string our_side;

    get_PARAM_NORMALISE_FIELD(should_normalize);
    get_PARAM_OUR_SIDE(our_side);


    if (!(should_normalize && our_side == "right")) {
        // No need to normalize.
        return world;
    }

    World norm_world(world);

    // Rotate the ball.
    norm_world.ball.pos.x *= -1;
    norm_world.ball.pos.y *= -1;

    norm_world.ball.vel.x *= -1;
    norm_world.ball.vel.y *= -1;


    return norm_world;
}

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
