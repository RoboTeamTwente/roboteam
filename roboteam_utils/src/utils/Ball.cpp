#include <roboteam_utils/Ball.hpp>

namespace rtt {

    bool Ball::operator==(const Ball &other) const {
        return this->position == other.position && this->height == other.height && this->velocity == other.velocity && this->verticalVelocity == other.verticalVelocity && this->expectedEndPosition == other.expectedEndPosition && this->isVisible == other.isVisible && this->area == other.area;
    }

}  // namespace rtt