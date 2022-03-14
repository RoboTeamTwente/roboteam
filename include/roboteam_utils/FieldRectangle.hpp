#pragma once

#include <Vector2.h>

namespace rtt {
/* A more optimized rectangle class that is used for representing a field.
   Optimized by precalculating many variables that are already known from
   the beginning, because field data is often read, but never changed. */
class FieldRectangle {
public:
    // Creates a simple unit rectangle between (0,0) and (1,1)
    constexpr FieldRectangle();
    // Creates a rectangle from a top, right, bottom and left boundary
    constexpr FieldRectangle(double top, double right, double bottom, double left);
    // Creates a rectangle from any two opposing corners
    constexpr FieldRectangle(const Vector2& pointA, const Vector2& pointB);

    bool operator==(const FieldRectangle& other) const;

    // Checks if the given point is inside the rectangle
    bool contains(const Vector2& point) const;

    double getTop() const;
    double getRight() const;
    double getBottom() const;
    double getLeft() const;
    double getWidth() const;
    double getHeight() const;
    const Vector2 getCenter() const;

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