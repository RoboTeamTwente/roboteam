//
// Created by rolf on 22-01-20.
//

#ifndef ROBOTEAM_UTILS_RECTANGLE_H
#define ROBOTEAM_UTILS_RECTANGLE_H
#include <vector>
#include "Vector2.h"
namespace rtt {
class LineSegment;
class Line;
class Polygon;

/**
 * @brief Represents a rectangle (with horizontal and vertical lines) by storing 2 opposite corners
 * @date 22-01-2020
 * @author Rolf van der Hulst
 */
class Rectangle {
   public:
    /**
     * @brief Constructs a rectangle from two opposite corners
     * @param corner Primary corner
     * @param oppositeCorner The corner which is opposite /diagonal from the other corner.
     */
    Rectangle(const Vector2 &corner, const Vector2 &oppositeCorner);
    /**
     * @brief Constructs a rectangle from the bottom left corner given a width and height.
     * @param bottomLeft The bottom left corner of the rectangle
     * @param x Distance the rectangle stretches in positive x-direction
     * @param y Distance the rectangle stretches in positive y-direction
     */
    Rectangle(const Vector2 &bottomLeft, double x, double y);
    Vector2 corner1;
    Vector2 corner2;

    /**
     * @return The smallest X value of the rectangle
     */
    [[nodiscard]] double minX() const;
    /**
     * @return The largest X value of the rectangle
     */
    [[nodiscard]] double maxX() const;
    /**
     * @return The smallest Y value of the rectangle
     */

    [[nodiscard]] double minY() const;
    /**
     * @return The largest Y value of the rectangle
     */
    [[nodiscard]] double maxY() const;

    /**
     * @brief Returns the width of the rectangle
     * @return The width of the rectangle
     */
    [[nodiscard]] double width() const;
    /**
     * @brief Returns the height of the rectangle
     * @return The height of the rectangle
     */
    [[nodiscard]] double height() const;

    /**
     * @brief Gets the absolute center (average) of the rectangle
     * @return The center point
     */
    [[nodiscard]] Vector2 center() const;
    /**
     * @brief Get the corners of the rectangle
     * @return A vector with the 4 corners of the rectangle
     */
    [[nodiscard]] std::vector<Vector2> corners() const;
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
    [[nodiscard]] bool contains(const Vector2 &point) const;
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

    /* 	 ______   _______  _______  ______     _______  _______  ______   _______
     *	(  __  \ (  ____ \(  ___  )(  __  \   (  ____ \(  ___  )(  __  \ (  ____ \
     *	| (  \  )| (    \/| (   ) || (  \  )  | (    \/| (   ) || (  \  )| (    \/
     *	| |   ) || (__    | (___) || |   ) |  | |      | |   | || |   ) || (__
     *	| |   | ||  __)   |  ___  || |   | |  | |      | |   | || |   | ||  __)
     *	| |   ) || (      | (   ) || |   ) |  | |      | |   | || |   ) || (
     *	| (__/  )| (____/\| )   ( || (__/  )  | (____/\| (___) || (__/  )| (____/\
     *	(______/ (_______/|/     \|(______/   (_______/(_______)(______/ (_______/
     *
     * The functions below are dead. Remove this tag if you use any of the functions and make sure to remove this tag at other places as well that will become alive by using any of the
     * function below. Do not read/document/redesign/analyse/test/optimize/etc. any of this code, because it is a waste of your time! This code was not removed or placed at another
     * branch, because other software developers are very attached to this code and are afraid that this code might be used at some day (but I think it won't be used at all and should
     * be removed).
     */

    /**
     * @brief Checks whether a given line intersects with `this`
     * @param line line to check for
     * @return True if `line` intersects with the rectangle
     */
    // [[nodiscard]] bool doesIntersect(const Line &line) const;

    /**
     * @brief Checks whether a given line intersects with `this`.
     * Not the most efficient and may return the same point twice if the line 'exactly' intersects with the corner.
     * In general prefer using LineSegments over Lines if you need intersections often.
     * @param line line segment to check for
     * @return vector with all of the points where the line segment intersects the rectangle
     */
    // [[nodiscard]] std::vector<Vector2> intersects(const Line &line) const;

};
// std::ostream &operator<<(std::ostream &out, const Rectangle &rect);

}  // namespace rtt

#endif  // ROBOTEAM_UTILS_RECTANGLE_H
