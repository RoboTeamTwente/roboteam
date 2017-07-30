#include "roboteam_world/danger_finder/DistanceModule.h"
#include "roboteam_msgs/GeometryFieldSize.h"

namespace rtt {
namespace df {

REGISTER_MODULE("Distance", DistanceModule)

DistanceModule::DistanceModule(double factor) : DangerModule("Distance"), factor(factor) {}

inline Vector2 getGoalCenter() {
	// static auto geom = LastWorld::get_field();
	auto goalPos = LastWorld::get_our_goal_center();
	return goalPos;
	// return Vector2(geom.field_length / 2, 0);
}

PartialResult DistanceModule::calculate(const roboteam_msgs::WorldRobot& bot, const roboteam_msgs::World& world) {
	double dist = Vector2(bot.pos).dist(getGoalCenter());
	// auto goalPos = LastWorld::get_our_goal_center(); 

	// std::cout << "goalCenter: " << goalPos.x << " " << goalPos.y << "\n";
	// std::cout << "robot " << bot.id 
		// << " pos: " << bot.pos.x << " " << bot.pos.y
		// << " dist: " << dist << "\n";
	double last = lastDistances[bot.id];
	DangerFlag flag;
	lastDistances[bot.id] = dist;

	if (last != 0.0 && last < dist) {
		flag = DANGER_CLOSING;
	} else {
        flag = DANGER_NONE;
    }

	return {(dist - myConfig().doubles["maxWidth"]) / factor, flag};
}


}
}
