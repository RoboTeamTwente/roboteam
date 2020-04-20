//
// Created by rolf on 18-4-19.
//

#include "../include/roboteam_utils/Line.h"
#include "../include/roboteam_utils/LineSegment.h"

namespace rtt {
Line::Line(const LineSegment &other) noexcept {
    start=other.start;
    end=other.end;
}
double Line::length() const {
    return (end - start).length();
}

double Line::length2() const {
    return (end - start).length2();
}

double Line::slope() const {
    return (end.y - start.y)/(end.x - start.x);
}

bool Line::isVertical() const {
    return (end.x == start.x) && (end.y != start.y);
}

Vector2 Line::direction() const {
    return Vector2(end - start);
}

double Line::intercept() const {
    return start.y - this->slope()*start.x;
}

std::pair<double, double> Line::coefficients() const {
    return {slope(), intercept()};
}

bool Line::isPoint() const {
    return start == end;
}

bool Line::isParallel(const Line &line) const {
    // check if line is vertical, otherwise check the slope
    if (this->isVertical() || line.isVertical()) {
        return this->isVertical() && line.isVertical();
    }
    return this->slope() == line.slope();
}
bool Line::isParallel(const LineSegment &line) const {
    // check if line is vertical, otherwise check the slope
    if (this->isVertical() || line.isVertical()) {
        return this->isVertical() && line.isVertical();
    }
    return this->slope() == line.slope();
}
double Line::distanceToLine(const Vector2 &point) const {
    return (this->project(point) - point).length();
}

///Computes the projection of point onto the line. This is identical to picking the closest point on the line
// if we project point P onto AB we can compute as A + dot(AP,AB) / dot(AB,AB) * AB
Vector2 Line::project(const Vector2 &point) const {
    if (start == end) return start;

    Vector2 AB = direction();
    Vector2 AP = point - start;
    return Vector2(start + AB*AP.dot(AB)/length2());
}

bool Line::isOnLine(const Vector2 &point) const {
    if (isPoint()){
        return (start == point || end == point);
    }
    Vector2 A = end - start;
    Vector2 B = point - start;
    return A.cross(B) == 0;
}

// see https://en.wikipedia.org/wiki/Line%E2%80%93line_intersection for help. These should be thoroughly tested
std::optional<Vector2> Line::intersects(const Line &line) const {
    auto result = generalIntersect(Line(line));
    return result.has_value() ? std::optional(result.value().first) : std::nullopt;
}

std::optional<Vector2> Line::intersects(const LineSegment &line) const {
    auto result = generalIntersect(Line(line));
    if (result.has_value()) {
        float u = result.value().second;
        if (u >= 0 && u <= 1) return std::optional(result.value().first);
    }
    return std::nullopt;
}

std::optional<Vector2> Line::forwardIntersect(const Line &line) const {
    auto result = generalIntersect(Line(line));
    if (result.has_value()) {
        float u = result.value().second;
        if (u >= 0) return std::optional(result.value().first);
    }
    return std::nullopt;
}

bool Line::doesIntersect(const Line &line) const {
    return !this->isParallel(line);
}

bool Line::doesIntersect(const LineSegment &line) const {
    return intersects(line).has_value();
}

std::optional<std::pair<Vector2, float>> Line::generalIntersect(const Line &line) const {
    Vector2 A = start - end;
    Vector2 B = line.start - line.end;
    double denom = A.cross(B);
    if (denom != 0) {
        Vector2 C = start - line.start;
        double numer = C.cross(A);
        double u = numer / denom;
        return std::pair(line.start - B*u, u);
    }
    return std::nullopt;
}

}
