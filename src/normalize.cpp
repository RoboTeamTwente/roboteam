#include "roboteam_utils/normalize.h"
#include "roboteam_utils/constants.h"

#include "roboteam_utils/Math.h"

#include <string>


namespace rtt {

using namespace roboteam_msgs;

DetectionFrame normalizeDetectionFrame(DetectionFrame frame) {
    std::string our_side;
    get_PARAM_OUR_SIDE(our_side);

    if (our_side == "right") {
        return rotateDetectionFrame(frame);
    } else {
        // No need to normalize.
        return frame;
    }
}

DetectionFrame rotateDetectionFrame(DetectionFrame frame) {
    for (auto& ball : frame.balls) {
        ball = rotateBall(ball);
    }

    for (auto& bot : frame.us) {
        bot = rotateRobot(bot);
    }

    for (auto& bot : frame.them) {
        bot = rotateRobot(bot);
    }

    return frame;
}

DetectionBall rotateBall(DetectionBall ball) {
    ball.pos.x *= -1;
    ball.pos.y *= -1;
    ball.pixel_pos.x *= -1;
    ball.pixel_pos.y *= -1;

    return ball;
}


DetectionRobot rotateRobot(DetectionRobot bot) {
    bot.pos.x *= -1;
    bot.pos.y *= -1;
    bot.orientation = cleanAngle(bot.orientation + M_PI);
    bot.pixel_pos.x *= -1;
    bot.pixel_pos.y *= -1;

    return bot;
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
