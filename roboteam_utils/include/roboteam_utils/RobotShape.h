#ifndef ROBOTEAM_UTILS_ROBOTSHAPE_H_
#define ROBOTEAM_UTILS_ROBOTSHAPE_H_

#include "Angle.h"
#include "Circle.h"
#include "Line.h"

namespace rtt {
class HalfLine;

class RobotShape {
   private:
    // The first three are necessary to determine the entire shape
    Circle circle;
    double centerToFront;
    Angle orientation;
    // The following is not necessary but prevents a lot of recomputation using sin() and cos() like functions which can be pretty expensive
    LineSegment kickerLine;

   public:
    RobotShape(const Vector2 &pos, double centerToFront, double radius, Angle orientation);

    /**
     * Checks if a given point lies in front (not on) of the line which defines the dribbler.
     * This only checks if the point is on the correct side of the halfplane defined by the line passing through the dribbler
     * @param point
     * @return
     */
    [[nodiscard]] bool inFrontOfDribbler(const Vector2 &point) const;

    [[nodiscard]] Vector2 pos() const;

    [[nodiscard]] Angle angle() const;

    [[nodiscard]] Vector2 centerOfKickerPos() const;

    [[nodiscard]] LineSegment kicker() const;

    [[nodiscard]] Vector2 project(const Vector2 &point) const;

    [[nodiscard]] double distanceTo(const Vector2 &point) const;

    [[nodiscard]] double squaredDistanceTo(const Vector2 &point) const;

    void move(const Vector2 &by);

    [[nodiscard]] bool contains(const Vector2 &point) const;

    [[nodiscard]] bool doesIntersect(const LineSegment &segment) const;

    // tries to find intersection points of a line segment with the robot. In some edge cases,
    // multiple points very close to eachother might be returned because of numerical errors.
    [[nodiscard]] std::vector<Vector2> intersects(const LineSegment &segment) const;

    [[nodiscard]] double radius() const;

    [[nodiscard]] double centerToFrontDist() const;
};

}  // namespace rtt
#endif  // ROBOTEAM_UTILS_ROBOTSHAPE_H_
