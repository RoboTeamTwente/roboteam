#include "roboteam_utils/world_analysis.h"

namespace rtt {

boost::optional<roboteam_msgs::WorldRobot> lookup_bot(unsigned int id, bool our_team, const roboteam_msgs::World* world) {
    const roboteam_msgs::World w = world == nullptr ? LastWorld::get() : *world;
    auto vec = our_team ? w.us : w.them;
    for (const auto& bot : vec) {
        if (bot.id == id) {
            return boost::optional<roboteam_msgs::WorldRobot>(bot);
        }
    }
    return boost::optional<roboteam_msgs::WorldRobot>();
}

boost::optional<roboteam_msgs::WorldRobot> lookup_our_bot(unsigned int id, const roboteam_msgs::World* world) {
    return lookup_bot(id, true, world);
}

boost::optional<roboteam_msgs::WorldRobot> lookup_their_bot(unsigned int id, const roboteam_msgs::World* world) {
    return lookup_bot(id, false, world);
}


bool bot_has_ball(const roboteam_msgs::WorldRobot& bot, const roboteam_msgs::WorldBall& ball) {
    Vector2 ball_vec(ball.pos.x, ball.pos.y), bot_vec(bot.pos.x, bot.pos.y);
    Vector2 ball_norm = (ball_vec - bot_vec);

    double dist = ball_norm.length();
    double angle = ball_norm.angle();

    // Within 10.5 cm and .2 radians (of center of dribbler)
    return dist <= .105 && fabs(angle - bot.angle) <= .2;
}

} // rtt
