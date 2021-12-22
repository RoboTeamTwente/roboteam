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

double sq(double x) { return x * x; }

std::vector<Vector2> Circle::intersectsCircleWithLineSegment(rtt::LineSegment line) {
    std::vector<rtt::Vector2> res;
    constexpr double eps = 1e-16;
    bool segment = true;

    double x0 = this->center.x;  // cp.first;
    double y0 = this->center.y;  // cp.second;
    double r = this->radius;
    double x1 = line.start.x;  // p1.first;
    double y1 = line.start.y;  // p1.second;
    double x2 = line.end.x;    // p2.first;
    double y2 = line.end.y;    // p2.second;
    double A = y2 - y1;
    double B = x1 - x2;
    double C = x2 * y1 - x1 * y2;
    double a = A * A + B * B;
    double b, c;
    bool bnz = true;
    if (abs(B) >= eps) {
        b = 2 * (A * C + A * B * y0 - sq(B) * x0);
        c = sq(C) + 2 * B * C * y0 - sq(B) * (sq(r) - sq(x0) - sq(y0));
    } else {
        b = 2 * (B * C + A * B * x0 - sq(A) * y0);
        c = sq(C) + 2 * A * C * x0 - sq(A) * (sq(r) - sq(x0) - sq(y0));
        bnz = false;
    }
    double d = sq(b) - 4 * a * c;  // discriminant
    if (d < 0) {
        return res;
    }

    // checks whether a point is within a segment
    auto within = [x1, y1, x2, y2](double x, double y) {
        double d1 = sqrt(sq(x2 - x1) + sq(y2 - y1));  // distance between end-points
        double d2 = sqrt(sq(x - x1) + sq(y - y1));    // distance from point to one end
        double d3 = sqrt(sq(x2 - x) + sq(y2 - y));    // distance from point to other end
        double delta = d1 - d2 - d3;
        return abs(delta) < eps;  // true if delta is less than a small tolerance
    };

    auto fx = [A, B, C](double x) { return -(A * x + C) / B; };

    auto fy = [A, B, C](double y) { return -(B * y + C) / A; };

    auto rxy = [segment, &res, within](double x, double y) {
        if (!segment || within(x, y)) {
            res.emplace_back(x, y);
        }
    };

    double x, y;
    if (d == 0.0) {
        // line is tangent to circle, so just one intersect at most
        if (bnz) {
            x = -b / (2 * a);
            y = fx(x);
            rxy(x, y);
        } else {
            y = -b / (2 * a);
            x = fy(y);
            rxy(x, y);
        }
    } else {
        // two intersects at most
        d = sqrt(d);
        if (bnz) {
            x = (-b + d) / (2 * a);
            y = fx(x);
            rxy(x, y);
            x = (-b - d) / (2 * a);
            y = fx(x);
            rxy(x, y);
        } else {
            y = (-b + d) / (2 * a);
            x = fy(y);
            rxy(x, y);
            y = (-b - d) / (2 * a);
            x = fy(y);
            rxy(x, y);
        }
    }
    return res;
}
}  // namespace rtt