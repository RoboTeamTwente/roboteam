#pragma once

#include <roboteam_utils/LineSegment.h>
#include <roboteam_utils/Polygon.h>
#include <roboteam_utils/Vector2.h>

#include <roboteam_utils/Rectangle.hpp>

namespace rtt {
/* A more optimized rectangle class that is used for representing a field.
   Optimized by precalculating many variables that are already known from
   the beginning, because field data is often read, but never changed. */
class FastRectangle : public Rectangle {
   public:
    // Creates a simple unit rectangle between (0,0) and (1,1)
    FastRectangle();
    // Creates a rectangle from a top, right, bottom and left boundary
    FastRectangle(double top, double right, double bottom, double left);
    // Creates a rectangle from any two opposing corners
    FastRectangle(const Vector2& pointA, const Vector2& pointB);

    bool operator==(const FastRectangle& other) const;

    // Checks if the given point is inside the rectangle
    bool contains(const Vector2& point) const override;
    // Checks if the given point is inside the rectangle, taken into account the margin from the outsides
    bool contains(const Vector2& point, double margin) const;

    // Projects the given point
    Vector2 project(const Vector2& point) const override;

    Polygon asPolygon(double margin = 0) const;
    double top() const override;
    double right() const override;
    double bottom() const override;
    double left() const override;
    double width() const override;
    double height() const override;
    Vector2 center() const override;
    Vector2 topLeft() const override;
    Vector2 topRight() const override;
    Vector2 bottomLeft() const override;
    Vector2 bottomRight() const override;
    LineSegment topLine() const override;
    LineSegment rightLine() const override;
    LineSegment bottomLine() const override;
    LineSegment leftLine() const override;

   private:
    double _top;
    double _right;
    double _bottom;
    double _left;
    double _width;
    double _height;
    Vector2 _center;
};

}  // namespace rtt