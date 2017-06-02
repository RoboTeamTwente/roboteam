#include "roboteam_utils/world_analysis.h"
#include <boost/range/join.hpp>

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

std::vector<roboteam_msgs::WorldRobot> getObstaclesBetweenPoints(const Vector2& bot_pos,
                                                    const Vector2& point,
                                                    const roboteam_msgs::World* world_ptr,
                                                    bool sight_only,
													bool ignore_both_ends) {
    const roboteam_msgs::World world = world_ptr == nullptr ? LastWorld::get() : *world_ptr;
    const auto all_bots = boost::join(world.us, world.them);

    double threshold = sight_only ? .15 : .35;

    std::vector<std::pair<roboteam_msgs::WorldRobot, double>> obstacles;
    for (const auto& obs : all_bots) {
        const Vector2 obs_pos(obs.pos.x, obs.pos.y);

        if (obs_pos == bot_pos || (ignore_both_ends && obs_pos == point)) continue;

        const Vector2 proj = obs_pos.project(bot_pos, point);
        double proj_dist = proj.dist(obs_pos);
        double dist_to_start = bot_pos.dist(obs_pos);
        if (proj_dist < threshold && dist_to_start > .0001) {
            obstacles.push_back(std::make_pair(obs, dist_to_start));
        }
    }

    auto sorter = [](const std::pair<roboteam_msgs::WorldRobot, double>& a,
                     const std::pair<roboteam_msgs::WorldRobot, double>& b) {
        return a.second < b.second;
    };
    std::sort<std::vector<std::pair<roboteam_msgs::WorldRobot, double>>::iterator, decltype(sorter)>
        (obstacles.begin(), obstacles.end(), sorter);
    std::vector<roboteam_msgs::WorldRobot> result;
    for (const auto& obs : obstacles) {
        result.push_back(obs.first);
    }
    return result;
}

} // rtt
