#pragma once

#include <roboteam_utils/Vector2.h>
#include <roboteam_utils/LineSegment.h>
#include <roboteam_utils/Polygon.h>

#include <roboteam_utils/Shape.h>

namespace rtt {
/* A more optimized rectangle class that is used for representing a field.
   Optimized by precalculating many variables that are already known from
   the beginning, because field data is often read, but never changed. */
class FieldRectangle : public Shape {
public:
    // Creates a simple unit rectangle between (0,0) and (1,1)
    FieldRectangle();
    // Creates a rectangle from a top, right, bottom and left boundary
    FieldRectangle(double top, double right, double bottom, double left);
    // Creates a rectangle from any two opposing corners
    FieldRectangle(const Vector2& pointA, const Vector2& pointB);

    bool operator==(const FieldRectangle& other) const;

    // Checks if the given point is inside the rectangle
    bool contains(const Vector2& point) const override;
    // Checks if the given point is inside the rectangle, taken into account the margin from the outsides
    bool contains(const Vector2& point, double margin) const;

    // Projects the given point
    Vector2 project(const Vector2& point) const override;

    Polygon asPolygon(double margin = 0) const;
    double getTop() const;
    double getRight() const;
    double getBottom() const;
    double getLeft() const;
    double getWidth() const;
    double getHeight() const;
    Vector2 getCenter() const;
    Vector2 getTopLeft() const;
    Vector2 getTopRight() const;
    Vector2 getBottomLeft() const;
    Vector2 getBottomRight() const;
    LineSegment getTopSide() const;
    LineSegment getRightSide() const;
    LineSegment getBottomSide() const;
    LineSegment getLeftSide() const;

private:
    double top;
    double right;
    double bottom;
    double left;
    double width;
    double height;
    Vector2 center;
};

} // namespace rtt