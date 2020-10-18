#include "Print.h"
#include "../include/roboteam_utils/Line.h"
#include "../include/roboteam_utils/LineSegment.h"

namespace rtt {
Line::Line(const Vector2 &v1, const Vector2 &v2) {
    this->v1 = v1;
    this->v2 = v2;
    if (v1 == v2) {
        RTT_WARNING("Warning: you created an undefined line, because v1 == v2. Note that Lines have an infinite length. If you want to have a Line with finite length then use "
                     "the LineSegment class instead.");
    }
}

Line::Line(const LineSegment &other) noexcept {
    v1 = other.start;
    v2 = other.end;
    if (v1 == v2) {
        RTT_WARNING("Warning: you created an undefined line, because v1 == v2. Note that Lines have an infinite length. If you want to have a Line with finite length then use "
                     "the LineSegment class instead.");
    }
}

double Line::distanceToLine(const Vector2 &point) const { return (this->project(point) - point).length(); }

Vector2 Line::project(const Vector2 &point) const {
    Vector2 AB = v2 - v1;
    Vector2 AP = point - v1;
    return v1 + AB * (AP.dot(AB) / AB.length2());
}

bool Line::isOnLine(const Vector2 &point) const {
    Vector2 A = v2 - v1;
    Vector2 B = point - v1;
    return abs(A.cross(B)) < RTT_PRECISION_LIMIT;
}

std::optional<Vector2> Line::intersect(const Line &line) const {
    auto result = intersect(v1, v2, line.v1, line.v2);
    if (result.has_value()) {
        return result;
    } else if (line.isOnLine(v1)) {
        return project({0, 0});
    } else {
        return std::nullopt;
    }
}

std::optional<Vector2> Line::intersect(const Vector2 p1, const Vector2 p2, const Vector2 q1, const Vector2 q2) {
    Vector2 A = p1 - p2;
    Vector2 B = q1 - q2;
    double denom = A.cross(B);
    if (abs(denom) >= RTT_PRECISION_LIMIT) {
        Vector2 C = p1 - q1;
        double numer = C.cross(A);
        double u = numer / denom;
        return q1 - B * u;
    }
    return std::nullopt;
}

float Line::relativePosition(Vector2 p1, Vector2 p2, Vector2 pointOnLine) {
    float xDiff = p2.x - p1.x;
    if (xDiff == 0.0) {
        return (pointOnLine.y - p1.y) / (p2.y - p1.y);
    } else {
        return (pointOnLine.x - p1.x) / xDiff;
    }
}

}  // namespace rtt
