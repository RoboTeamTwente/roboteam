#include "HalfLine.h"

namespace rtt {
HalfLine::HalfLine(const Vector2 &start, const Vector2 &goesThrough) {
    this->start = start;
    this->goesThrough = goesThrough;
    if (start == goesThrough) {
        std::cerr << "Warning: you created an undefined line, because start == goesThrough. Note that HalfLines have an infinite length. If you want to have a Line with finite "
                     "length then use the LineSegment class instead."
                  << std::endl;
    }
}

std::optional<Vector2> HalfLine::intersect(const Line &line) {
    auto result = Line::intersect(start, goesThrough, line.v1, line.v2);
    if (result.has_value()) {
        float t = Line::relativePosition(start, goesThrough, result.value());
        if (t >= 0) return result.value();
    } else if (line.isOnLine(start)) {
        return project(Vector2(0, 0));
    }
    return std::nullopt;
}

Vector2 HalfLine::project(const Vector2 &point) const {
    Vector2 projection = Line(start, goesThrough).project(point);
    double t = Line::relativePosition(start, goesThrough, projection);
    if (t < 0) {
        return start;
    }
    return projection;
}

}  // namespace rtt
