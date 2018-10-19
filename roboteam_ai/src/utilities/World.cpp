#include "World.h"

namespace rtt {
namespace ai {

// define the static variables
roboteam_msgs::World World::world;

const roboteam_msgs::World &World::get_world() {
  return World::world;
}

void World::set_world(roboteam_msgs::World world) {
  World::world = world;
}

boost::optional<roboteam_msgs::WorldRobot> World::getWorldBot(unsigned int id, bool robotIsOurTeam) {
  const std::vector<roboteam_msgs::WorldRobot>& robots = robotIsOurTeam ? world.us : world.them;
  for (const auto& bot : robots) {
    if (bot.id == id) {
      return boost::optional<roboteam_msgs::WorldRobot>(bot);
    }
  }
  return boost::none;
}

} // ai
} // rtt