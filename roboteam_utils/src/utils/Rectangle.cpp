#include <roboteam_utils/Rectangle.hpp>

namespace rtt {

void Rectangle::addPointsToVec(std::vector<Vector2>& vec) const {
    vec.push_back(this->center());
}

std::vector<Vector2> Rectangle::getPoints() const {
    std::vector<Vector2> points;
    this->addPointsToVec(points);
    return points;
}

} // namespace rtt
