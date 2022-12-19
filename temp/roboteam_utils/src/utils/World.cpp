#include <roboteam_utils/World.hpp>

#include <google/protobuf/util/message_differencer.h> // For comparing proto objects

namespace rtt {

bool World::operator==(const World &other) const {
    return this->timePoint == other.timePoint
        && this->id == other.id
        && this->ball == other.ball
        && this->yellowRobots == other.yellowRobots
        && this->blueRobots == other.blueRobots;
}

bool WorldStates::operator==(const WorldStates &other) const {
    return this->currentWorld == other.currentWorld
        && this->field == other.field;
}

} // namespace rtt