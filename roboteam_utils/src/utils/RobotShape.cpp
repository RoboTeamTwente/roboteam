#include "RobotShape.h"

#include <cassert>

#include "Print.h"

namespace rtt {
RobotShape::RobotShape(const Vector2 &pos, double centerToFront, double radius, Angle orientation)
    : circle{Circle(pos, radius)}, centerToFront{centerToFront}, orientation{orientation} {
    assert(centerToFront <= radius);
    // Slightly more complicated, but we do so to minimize the amount of times we need to call cos() and sin()
    Vector2 middleToCenter = Vector2(orientation).stretchToLength(centerToFront);
    Vector2 dribblerCenter = pos + middleToCenter;
    double halfFrontWidth = sqrt(radius * radius - centerToFront * centerToFront);  // Pythagoras
    Vector2 diff = Vector2(-middleToCenter.y, middleToCenter.x).stretchToLength(halfFrontWidth);
    kickerLine = LineSegment(dribblerCenter - diff, dribblerCenter + diff);  // Lower corner, upper corner (when robot is facing right (e.g. yaw = 0))
}

bool RobotShape::inFrontOfDribbler(const Vector2 &point) const {
    // We use the cross product to determine which side of the plane the point is on.
    // KickerLine is always oriented such that this definition is consistent with the ball being ahead of the robot.
    // Note we don't do >=0 because we want to exclude the case where the point is exactly on the line.
    return kickerLine.direction().cross(kickerLine.start - point) > 0;
}

void RobotShape::move(const Vector2 &by) {
    circle.move(by);
    kickerLine.move(by);
}

bool RobotShape::contains(const Vector2 &point) const { return circle.contains(point) && !inFrontOfDribbler(point); }

bool RobotShape::doesIntersect(const LineSegment &segment) const {
    std::vector<Vector2> intersects = circle.intersects(segment);
    if (intersects.empty()) {
        // No intersections with circle, which means segment is either completely outside or completely inside the circle
        //  We need to check for the case where the segment is completely inside and hits the dribbler
        return circle.contains(segment.start) && circle.contains(segment.end) && kickerLine.preciseDoesIntersect(segment);
    } else if (intersects.size() == 1) {
        // Either the segment touches the circle, or (more likely) it starts or ends inside of it.
        if (inFrontOfDribbler(intersects[0])) {
            // the segment might still intersect with the kicker, if not, there is no intersection
            return kickerLine.preciseDoesIntersect(segment);
        }
        // The intersection point is on the hull
        return true;
    } else if (intersects.size() == 2) {
        bool firstInFront = inFrontOfDribbler(intersects[0]);
        bool secondInFront = inFrontOfDribbler(intersects[1]);
        return !(firstInFront && secondInFront);
    }
    return false;
}

std::vector<Vector2> RobotShape::intersects(const LineSegment &segment) const {
    std::vector<Vector2> intersects = circle.intersects(segment);
    if (intersects.empty()) {
        // No intersections with circle, which means segment is either completely outside or completely inside the circle
        //  We need to check for the case where the segment is completely inside and hits the dribbler
        if (circle.contains(segment.start) && circle.contains(segment.end)) {  // one of these checks should technically be redundant. But not optimizing for now.
            // then we simply compute intersections of the kicker line with the line segment
            auto intersect = kickerLine.firstIntersects(segment);
            if (intersect) {
                return {*intersect};
            }
        }
        // The segment is completely outside the circle.
        return {};
    } else if (intersects.size() == 1) {
        auto intersect = segment.firstIntersects(kickerLine);  // we need this info in all branches
        // Either the segment touches the circle, or (more likely) it starts or ends inside of it.
        if (inFrontOfDribbler(intersects[0])) {
            // The segment might still intersect with the kicker
            if (intersect) {
                return {*intersect};
            }
            return {};
        }
        // If the intersection point is on the hull there is no problem. However, there might be another intersection at the kicker.
        if (intersect) {
            // Check which point was the first collision
            if ((segment.start - *intersect).length2() < (segment.start - intersects[0]).length2()) {
                return {*intersect, intersects[0]};
            }
            intersects.push_back(*intersect);
        }
        return intersects;
    } else if (intersects.size() == 2) {
        bool firstInFront = inFrontOfDribbler(intersects[0]);
        bool secondInFront = inFrontOfDribbler(intersects[1]);
        // Check where both points are on the circle:
        if (firstInFront && secondInFront) {
            // no intersections as both pass in front of the robot
            return {};
        }
        if (!firstInFront && !secondInFront) {
            // Both points intersect the outer hull and not the dribbler. So we can simply return
            return intersects;
        }
        // One point is in front and one point is behind the kick line.
        // As the circle is convex, this must mean there is an intersection between the two.
        Vector2 dribblerIntersect = *kickerLine.firstIntersects(segment);
        // Check which of the intersections was invalid.
        if (firstInFront) {
            return {dribblerIntersect, intersects[1]};
        }
        return {intersects[0], dribblerIntersect};
    }
    RTT_DEBUG("bad robot intersection, should not be hit (numerical problems?)");
    return {};  // This should never be hit as circle intersection always hits atleast two.
}

Angle RobotShape::yaw() const { return orientation; }

Vector2 RobotShape::pos() const { return circle.center; }

Vector2 RobotShape::centerOfKickerPos() const { return (kickerLine.start + kickerLine.end) * 0.5; }

LineSegment RobotShape::kicker() const { return kickerLine; }

Vector2 RobotShape::project(const Vector2 &point) const {
    Vector2 circleProjection = circle.project(point);
    Vector2 lineProjection = kickerLine.project(point);
    if (inFrontOfDribbler(circleProjection)) {
        return lineProjection;
    }
    return (circleProjection - point).length2() <= (lineProjection - point).length2() ? circleProjection : lineProjection;
}

double RobotShape::distanceTo(const Vector2 &point) const { return (project(point) - point).length(); }

double RobotShape::squaredDistanceTo(const Vector2 &point) const { return (project(point) - point).length2(); }

double RobotShape::radius() const { return circle.radius; }

double RobotShape::centerToFrontDist() const { return centerToFront; }
}  // namespace rtt