//
// Created by rolf on 24-4-19.
//

#ifndef ROBOTEAM_UTILS_POLYGON_H
#define ROBOTEAM_UTILS_POLYGON_H

#include "Vector2.h"
#include "LineSegment.h"
#include <iostream>
#include <vector>

namespace rtt {

    class Polygon {
    public:
        /**
         * @brief Vertices that belong to this polygon
         * 
         */
        std::vector<Vector2> vertices;

        /**
         * @brief Construct a new Polygon object
         *
         * It's a rectangular polygon that's placed horizontally
         * 
         * @param lowerLeftCorner Lower left corner of the polygon
         * @param xlen X-axis length of the polygon
         * @param ylen Y-axis length of the polygon
         */
        Polygon(const Vector2 &lowerLeftCorner, double xlen, double ylen);

        /**
         * @brief Construct a new Polygon object
         * 
         * @param vertices Vertices that will literally be copied to this->vertices
         */
        Polygon(const std::vector<Vector2> &vertices);

        /**
         * @brief Moves a polygon
         * 
         * @param moveBy Moves the polygon by this much
         */
        void move(const Vector2 &moveBy);

        /**
         * @brief Gets the centroid of this polygon
         * 
         * @return Vector2 Vector representation of the centroid
         */
        [[nodiscard]] Vector2 centroid() const;

        /**
         * @brief The centroid of the set of vertices. 
         * 
         * Is generally NOT the same as centroid for polygons with more than 3 sides
         * 
         * @return Vector2 Vector representation of the centroid
         */
        [[nodiscard]] Vector2 verticeCentroid() const;

        /**
         * @brief Returns the vector at index \ref idx
         * 
         * @param idx Index in vector
         * @return Vector2 Vector that is at index \ref
         */
        [[nodiscard]] Vector2 operator[](size_t idx) const;

        /**
         * @brief Returns the amount of vertices in this polygon
         * 
         * Note: O(1)
         * 
         * @return size_t Amount of vertices 
         */
        [[nodiscard]] size_t amountOfVertices() const;

        /**
         * @brief Get the boundary of the polygon
         * 
         * @return std::vector<LineSegment> A vector of LineSegments which represent the boundary
         */
        [[nodiscard]] std::vector<LineSegment> getBoundary() const;

        /**
         * @brief Gets the length of the perimeter of the current polygon
         * 
         * @return double Gets the length of the perimeter
         */
        [[nodiscard]] double perimeterLength() const;

        /**
         * @brief Checks whether this is a convex hull
         * 
         * @return true True if it's a convex
         * @return false False if not a convex
         */
        [[nodiscard]] bool isConvex() const;

        /**
         * @brief Checks whether the polygon is simple
         * 
         * There are multiple possible algorithms, see
         * https://www.quora.com/What-is-the-simplest-algorithm-to-know-if-a-polygon-is-simple-or-not
         * this is the 'naive' O(N^2) approach which is fine for small cases (polygons with less than say 8-10 vertices)
         * 
         * @return true True if simple
         * @return false False if not simple
         */
        [[nodiscard]] bool isSimple() const;

        /**
         * @brief Checks whether a point is contained within this polygon
         * 
         * @param point Point to check
         * @return true True if contained
         * @return false False if not contained
         */
        [[nodiscard]] bool contains(const Vector2 &point) const;

        /**
         * @brief Checks whether a point is on the boundary of a polygon
         * 
         * @param point Point to check
         * @return true True if it's on boundary
         * @return false False if it's not on boundary
         */
        [[nodiscard]] bool isOnBoundary(const Vector2 &point) const;

        /**
         * @brief Checks whether a Line intersects a polygon
         * 
         * @param line Line to check
         * @return true True if it's intersecting
         * @return false False if it's not intersecting
         */
        [[nodiscard]] bool doesIntersect(const LineSegment &line) const;

        /**
         * @brief Gets intersections with a polygon and line 
         * 
         * @param line Line to get intersections with
         * @return std::vector<Vector2> Vector filled with vector representations of the intersections
         */
        [[nodiscard]] std::vector<Vector2> intersections(const LineSegment &line) const;

        /**
         * @brief Gets the area of a polygon
         * 
         * @return double Area of polygon
         */
        [[nodiscard]] double area() const;

        /**
         * @brief Gets the area of a polygon
         * 
         * @return double Area of the polygon
         */
        [[nodiscard]] double doubleSignedArea() const;


    };
}
#endif //ROBOTEAM_UTILS_POLYGON_H
