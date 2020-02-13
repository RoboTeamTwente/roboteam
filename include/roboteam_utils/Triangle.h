//
// Created by rolf on 24-01-20.
//

#ifndef RTT_TRIANGLE_H
#define RTT_TRIANGLE_H
#include "Vector2.h"
namespace rtt {
class LineSegment;
class Line;
/**
 * @author rolf
 * @date 28 January 2020
 * @brief A class that describes a triangle and some useful functions on it.
 */
class Triangle {
    public:
        /**
         * @brief Construct a triangle from 3 points
         * @param point1 First point
         * @param point2 Second point
         * @param point3 Third point
         */
        Triangle(const Vector2 &point1, const Vector2 &point2, const Vector2 &point3);

        /**
         * The three points that define the corners of the triangle
         */
        Vector2 corner1;
        Vector2 corner2;
        Vector2 corner3;
        /**
         * @brief Checks whether a point is inside or on the boundary of the triangle
         * @param point
         * @return True if point is inside or on the boundary of the triangle
         */
        [[nodiscard]] bool contains(const Vector2 &point) const;
        /**
         * @brief Get the area of the triangle
         * @return The area of the triangle
         */
        [[nodiscard]] double area() const;
        /**
         * @brief Get the boundary lines of the triangle
         * @return A vector containing the 3 line segments that define the boundary of the triangle
         */
        [[nodiscard]] std::vector<LineSegment> lines() const;
        /**
         * @brief Get the corners of the triangle
         * @return A vector containing the 3 corners of the triangle
         */
        [[nodiscard]] std::vector<Vector2> corners() const;
        /**
         * @brief Get the intersection points of the line and triangle.
         * @param line line to check intersections for
         * @return A vector with all the intersection points. May have duplicate corner points.
         */
        [[nodiscard]] std::vector<Vector2> intersects(const LineSegment &line) const;
        /**
         * @brief Get the intersection points of the line and triangle.
         * @param line line to check intersections for
         * @return A vector with all the intersection points. May have duplicate corner points.
         */
        [[nodiscard]] std::vector<Vector2> intersects(const Line &line) const;
        /**
         * @brief checks if the line intersects with the triangle
         * @param line
         * @return True if the line intersects with the triangle
         */
        [[nodiscard]] bool doesIntersect(const LineSegment &line) const;
        /**
         * @brief checks if the line intersects with the triangle
         * @param line
         * @return True if the line intersects with the triangle
         */
        [[nodiscard]] bool doesIntersect(const Line &line) const;
};
}
#endif //RTT_TRIANGLE_H
