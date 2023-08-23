//
// Created by emiel on 24-02-20.
//

#include "Circle.h"

#include "Polynomial.h"

namespace rtt {

bool Circle::doesIntersectOrContain(const Vector2 &other) { return fabs((center - other).length()) <= radius; }

bool Circle::doesIntersectOrContain(const Line &other) { return other.distanceToLine(center) <= radius; }

bool Circle::doesIntersectOrContain(const LineSegment &other) { return other.distanceToLine(center) <= radius; }

bool Circle::doesIntersectOrContain(const Circle &other) { return fabs((center - other.center).length()) <= (radius + other.radius); }

bool Circle::doesIntersectOrContain(const Rectangle &other) {
    // https://stackoverflow.com/questions/401847/circle-rectangle-collision-detection-intersection

    Vector2 rectCenter = other.center();
    Vector2 distanceToCircle;

    double rectWidth = other.width();
    double rectHeight = other.height();

    distanceToCircle.x = fabs(center.x - rectCenter.x);
    distanceToCircle.y = fabs(center.y - rectCenter.y);

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

    std::vector<LineSegment> segments = {other.leftLine(), other.topLine(), other.rightLine(), other.bottomLine()};
    for (const LineSegment &segment : segments)
        if (doesIntersectOrContain(segment)) return true;

    return false;
}

Vector2 Circle::project(const Vector2 &point) const { return center + (point - center).stretchToLength(radius); }

bool Circle::operator==(const Circle &other) const { return center == other.center && radius == other.radius; }
bool Circle::operator!=(const Circle &other) const { return !(*this == other); }

Circle Circle::operator+(const Vector2 &other) const { return {center + other, radius}; }
Circle Circle::operator-(const Vector2 &other) const { return {center - other, radius}; }
Circle Circle::operator+=(const Vector2 &other) { return {center += other, radius}; }
Circle Circle::operator-=(const Vector2 &other) { return {center -= other, radius}; }

Circle Circle::operator*(double scale) const { return {center, radius * fabs(scale)}; }
Circle Circle::operator/(double scale) const { return {center, radius / fabs(scale)}; }
Circle Circle::operator*=(double scale) { return {center, radius *= fabs(scale)}; }
Circle Circle::operator/=(double scale) { return {center, radius /= fabs(scale)}; }

std::ostream &Circle::write(std::ostream &os) const { return os << "Circle({" << center.x << ", " << center.y << "}, " << radius << ")"; }
bool Circle::contains(const Vector2 &point) const { return (center - point).length2() <= radius * radius; }

void Circle::move(const Vector2 &by) { center += by; }

// https://stackoverflow.com/questions/1073336/circle-line-segment-collision-detection-algorithm
std::vector<Vector2> Circle::intersects(const LineSegment &segment) const {
    Vector2 d = segment.direction();
    Vector2 f = segment.start - center;

    double a = d.dot(d);
    double b = 2 * f.dot(d);
    double c = f.dot(f) - radius * radius;
    // Note the values here are sorted ascending already
    std::optional<std::pair<double, double>> values = solveQuadraticPositiveA(a, b, c);
    if (!values) {
        return {};
    }
    std::vector<Vector2> intersections;
    if (values->first >= 0 && values->first <= 1) {
        intersections.push_back(segment.start + d * values->first);
    }
    if (values->second >= 0 && values->second <= 1 && values->second != values->first) {
        intersections.push_back(segment.start + d * values->second);
    }
    return intersections;
}
std::ostream &operator<<(std::ostream &os, rtt::Circle const &circle) { return circle.write(os); }

double sq(double x) { return x * x; }

}  // namespace rtt