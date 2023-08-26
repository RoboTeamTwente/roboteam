#pragma once

#include "LineSegment.h"
#include "Shape.h"
#include "Vector2.h"

namespace rtt {

class Rectangle : public Shape {
   public:
    // Gets the width of the rectangle
    [[nodiscard]] virtual double width() const = 0;
    // Gets the height of the rectangle
    [[nodiscard]] virtual double height() const = 0;

    // Gets the largest y value
    [[nodiscard]] virtual double top() const = 0;
    // Gets the smallest x value
    [[nodiscard]] virtual double left() const = 0;
    // Gets the largest x value
    [[nodiscard]] virtual double right() const = 0;
    // Gets the smallest y value
    [[nodiscard]] virtual double bottom() const = 0;

    // Gets the center of the rectangle
    [[nodiscard]] virtual Vector2 center() const = 0;
    // Gets the top left corner
    [[nodiscard]] virtual Vector2 topLeft() const = 0;
    // Gets the top right corner
    [[nodiscard]] virtual Vector2 topRight() const = 0;
    // Gets the bottom left corner
    [[nodiscard]] virtual Vector2 bottomLeft() const = 0;
    // Gets the bottom right corner
    [[nodiscard]] virtual Vector2 bottomRight() const = 0;
    // Gets the top line segment, points in clockwise rotation
    [[nodiscard]] virtual LineSegment topLine() const = 0;
    // Gets the left line segment, points in clockwise rotation
    [[nodiscard]] virtual LineSegment leftLine() const = 0;
    // Gets the right line segment, points in clockwise rotation
    [[nodiscard]] virtual LineSegment rightLine() const = 0;
    // Gets the bottom line segment, points in clockwise rotation
    [[nodiscard]] virtual LineSegment bottomLine() const = 0;
};

}  // namespace rtt