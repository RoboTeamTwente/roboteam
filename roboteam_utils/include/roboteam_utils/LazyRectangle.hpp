#pragma once
#include <vector>

#include "Rectangle.hpp"
#include "Vector2.h"

namespace rtt {
    class LineSegment;
    class Line;
    class Polygon;

    /**
     * @brief Represents a rectangle (with horizontal and vertical lines) by storing 2 opposite corners
     * All other properties are calculated everytime in their functions (hence, a lazy rectangle)
     * @date 22-01-2020
     * @author Rolf van der Hulst
     */
    class LazyRectangle : public Rectangle {
    public:
        /**
         * @brief Constructs a rectangle from two opposite corners
         * @param corner Primary corner
         * @param oppositeCorner The corner which is opposite /diagonal from the other corner.
         */
        LazyRectangle(const Vector2 &corner, const Vector2 &oppositeCorner);
        /**
         * @brief Constructs a rectangle from the bottom left corner given a width and height.
         * @param bottomLeft The bottom left corner of the rectangle
         * @param x Distance the rectangle stretches in positive x-direction
         * @param y Distance the rectangle stretches in positive y-direction
         */
        LazyRectangle(const Vector2 &bottomLeft, double x, double y);
        Vector2 corner1;
        Vector2 corner2;

        /**
         * @return The smallest X value of the rectangle
         */
        [[nodiscard]] double left() const override;
        /**
         * @return The largest X value of the rectangle
         */
        [[nodiscard]] double right() const override;
        /**
         * @return The smallest Y value of the rectangle
         */
        [[nodiscard]] double bottom() const override;
        /**
         * @return The largest Y value of the rectangle
         */
        [[nodiscard]] double top() const override;

        /**
         * @brief Returns the width of the rectangle
         * @return The width of the rectangle
         */
        [[nodiscard]] double width() const override;
        /**
         * @brief Returns the height of the rectangle
         * @return The height of the rectangle
         */
        [[nodiscard]] double height() const override;

        /**
         * @brief Gets the absolute center (average) of the rectangle
         * @return The center point
         */
        [[nodiscard]] Vector2 center() const override;

        [[nodiscard]] Vector2 topLeft() const override;
        [[nodiscard]] Vector2 topRight() const override;
        [[nodiscard]] Vector2 bottomLeft() const override;
        [[nodiscard]] Vector2 bottomRight() const override;

        /**
         * @brief Get the corners of the rectangle
         * @return A vector with the 4 corners of the rectangle
         */
        [[nodiscard]] std::vector<Vector2> corners() const;

        LineSegment topLine() const override;
        LineSegment leftLine() const override;
        LineSegment rightLine() const override;
        LineSegment bottomLine() const override;
        /**
         * @brief Creates the 4 line segments as defined by the corners of this rectangle
         * @return A vector with 4 line segments which define the boundary of this rectangle
         */
        [[nodiscard]] std::vector<LineSegment> lines() const;
        /**
         * @brief Creates a polygon from the corners of the rectangle
         * @return A polygon of all four points
         */
        [[nodiscard]] Polygon asPolygon() const;
        /**
         * @brief Checks whether a given line intersects with `this`
         * @param line line segment to check for
         * @return vector with all of the points where the line segment intersects the rectangle
         */
        [[nodiscard]] std::vector<Vector2> intersects(const LineSegment &line) const;
        /**
         * @brief Checks whether a given line segment intersects with `this`
         * @param line line segment to check for
         * @return True if `line` intersects with the rectangle
         */
        [[nodiscard]] bool doesIntersect(const LineSegment &line) const;
        /**
         * @brief Checks whether given point is inside `this`
         * @param point point to check
         * @return True if point is inside or on boundary of the rectangle
         */
        [[nodiscard]] bool contains(const Vector2 &point) const override;
        /**
         * @brief Projects the given point, always resulting in a point inside
         * @param point the point to be projected
         * @return the result of the projection
         */
        [[nodiscard]] Vector2 project(const Vector2 &point) const override;
        /**
         * @brief This method is used internally for lineSegment intersection. It checks whether
         * @param point point to check for
         * @return the CS-code for that point (which contains info about the relative position of the point)
         */
        [[nodiscard]] unsigned int CohenSutherlandCode(const Vector2 &point) const;
        /**
         * @brief Writes a textual representation of this rectangle to the given output stream.
         */
        std::ostream &write(std::ostream &out) const;

        /**
         * @brief Checks whether a given line intersects with `this`
         * @param line line to check for
         * @return True if `line` intersects with the rectangle
         */
        [[nodiscard]] bool doesIntersect(const Line &line) const;

        /**
         * @brief Checks whether a given line intersects with `this`.
         * Not the most efficient and may return the same point twice if the line 'exactly' intersects with the corner.
         * In general prefer using LineSegments over Lines if you need intersections often.
         * @param line line segment to check for
         * @return vector with all of the points where the line segment intersects the rectangle
         */
        [[nodiscard]] std::vector<Vector2> intersects(const Line &line) const;
    };
    std::ostream &operator<<(std::ostream &out, const LazyRectangle &rect);

}  // namespace rtt
