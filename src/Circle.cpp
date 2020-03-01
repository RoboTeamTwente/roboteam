//
// Created by emiel on 24-02-20.
//

#include "Circle.h"

namespace rtt {

bool Circle::doesIntersectOrContain(const Vector2 &other) { return std::abs((center - other).length()) <= radius; }

bool Circle::doesIntersectOrContain(const Line &other) { return other.distanceToLine(center) <= radius; }

bool Circle::doesIntersectOrContain(const LineSegment &other) { return other.distanceToLine(center) <= radius; }

bool Circle::doesIntersectOrContain(const Circle &other) { return std::abs((center - other.center).length()) <= (radius + other.radius); }

bool Circle::doesIntersectOrContain(const Rectangle &other) {
    // https://stackoverflow.com/questions/401847/circle-rectangle-collision-detection-intersection

    Vector2 rectCenter = other.center();
    Vector2 distanceToCircle;

    double rectWidth = other.width();
    double rectHeight = other.height();

    distanceToCircle.x = abs(center.x - rectCenter.x);
    distanceToCircle.y = abs(center.y - rectCenter.y);

    if (distanceToCircle.x > (rectWidth / 2 + radius)) return false;
    if (distanceToCircle.y > (rectHeight / 2 + radius)) return false;

    if (distanceToCircle.x <= (rectWidth / 2)) return true;
    if (distanceToCircle.y <= (rectHeight / 2)) return true;

    double cornerDistance_sq = std::pow((distanceToCircle.x - rectWidth / 2), 2) + std::pow((distanceToCircle.y - rectHeight / 2), 2);

    return (cornerDistance_sq <= (radius * radius));
}

bool Circle::doesIntersectOrContain2(const Rectangle &other) {
    // https://stackoverflow.com/questions/401847/circle-rectangle-collision-detection-intersection

    if (other.contains(center)) return true;

    std::vector<LineSegment> segments = other.lines();
    for (const LineSegment &segment : segments)
        if (doesIntersectOrContain(segment)) return true;

    return false;
}

Vector2 Circle::project(const Vector2 &point) { return center + (point - center).stretchToLength(radius); }

bool Circle::operator==(const Circle &other) const { return center == other.center && radius == other.radius; }
bool Circle::operator!=(const Circle &other) const { return !(*this == other); }

Circle Circle::operator+(const Vector2 &other) const { return {center + other, radius}; }
Circle Circle::operator-(const Vector2 &other) const { return {center - other, radius}; }
Circle Circle::operator+=(const Vector2 &other) { return {center += other, radius}; }
Circle Circle::operator-=(const Vector2 &other) { return {center -= other, radius}; }

Circle Circle::operator*(double scale) const { return {center, radius * std::abs(scale)}; }
Circle Circle::operator/(double scale) const { return {center, radius / std::abs(scale)}; }
Circle Circle::operator*=(double scale) { return {center, radius *= std::abs(scale)}; }
Circle Circle::operator/=(double scale) { return {center, radius /= std::abs(scale)}; }

std::ostream &Circle::write(std::ostream &os) const { return os << "Circle({" << center.x << ", " << center.y << "}, " << radius << ")"; }

std::ostream &operator<<(std::ostream &os, rtt::Circle const &circle) { return circle.write(os); }
}  // namespace rtt