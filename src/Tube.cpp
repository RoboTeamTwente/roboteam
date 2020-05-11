//
// Created by rolf on 24-02-20.
//

#include <Circle.h>
#include "Tube.h"
namespace rtt{
bool Tube::contains(const Vector2 &point) const {
    return lineSegment.distanceToLine(point)<=radius; // If the point is within radius of the line the tube contains the point.
}
bool Tube::doesIntersectOrContain(const LineSegment &line) const {
    return lineSegment.distanceToLine(line)<=radius; // If the distance between the two lines is smaller than radius the line intersects the tube
}
bool Tube::isCircle() const {
    return lineSegment.isPoint();
}
Tube::Tube(const LineSegment& line, double radius) : lineSegment{line}, radius{radius} {
}
Tube::Tube(const Vector2& start, const Vector2& end, double radius) : lineSegment{LineSegment(start,end)},radius{radius} {
}
Tube::Tube() : lineSegment{},radius{1.0}{ // lines default construct to (0,0) to (0,0)
}
bool Tube::doesIntersectOrContain(const Circle &circle) const {
    return lineSegment.distanceToLine(circle.center)<=(radius+circle.radius);
}
Vector2 Tube::project(const Vector2 &point) {
    auto projectedPoint = lineSegment.project(point);
    return projectedPoint + (point - projectedPoint).stretchToLength(radius);
}
}


