#pragma once

#include "roboteam_msgs/WorldRobot.h"
#include "roboteam_msgs/WorldBall.h"
#include "roboteam_msgs/World.h"
#include "roboteam_utils/LastWorld.h"
#include "boost/optional.hpp"

namespace rtt {

/**
 * Looks up the given bot on the given team in the given world, and returns an optional WorldRobot.
 * If you don't pass a pointer to world, the function will get a world through LastWorld.
 */
boost::optional<roboteam_msgs::WorldRobot> lookup_bot(unsigned int id, bool our_team, const roboteam_msgs::World* world = nullptr);
boost::optional<roboteam_msgs::WorldRobot> lookup_our_bot(unsigned int id, const roboteam_msgs::World* world = nullptr);
boost::optional<roboteam_msgs::WorldRobot> lookup_their_bot(unsigned int id, const roboteam_msgs::World* world = nullptr);

bool bot_has_ball(const roboteam_msgs::WorldRobot& bot, const roboteam_msgs::WorldBall& ball);

bool bot_has_ball(unsigned int id, bool our_team, const roboteam_msgs::WorldBall& ball);

std::vector<roboteam_msgs::WorldRobot> getObstaclesBetweenPoints(const Vector2& bot_pos,
                                                    const Vector2& point,
                                                    const roboteam_msgs::World* world_ptr = nullptr,
                                                    bool sight_only = true,
													bool ignore_both_ends = false);

} // rtt
