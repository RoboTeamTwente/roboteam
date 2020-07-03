#ifndef ROBOTEAM_UTILS_LINE_H
#define ROBOTEAM_UTILS_LINE_H

#include "Vector2.h"

namespace rtt {
class LineSegment;

/**
 * The Line class represents an infinite length line, it is encoded by two different points through which the line is infinitely extended in both directions. If you want to use
 * a finite length line instead then use LineSegment class instead.
 * @author Created by: Rolf van der Hulst <br>
 *         Documented and redesigned by: Haico Dorenbos
 * @since 2019-04-18
 */
class Line {
   public:
    Vector2 v1;  // One of the points located on the Line.
    Vector2 v2;  // Another point located on the Line, which should be different than the v1.

    /**
     * Creates a new Line instance by using 2 different points on that Line. Make sure that the given points are different.
     *
     * @param v1 An arbitrary point on the Line.
     * @param v2 An arbitrary different point on the Line (make sure that it is different than the v2 point).
     */
    explicit Line(const Vector2 &v1, const Vector2 &v2);

    /**
     * Create a new Line from a LineSegment by infinitely expanding that LineSegment.
     *
     * @param other LineSegment that is infinitely expanded. Make sure that the LineSegment has a length > 0.
     */
    explicit Line(const LineSegment &other) noexcept;

    /**
     * Computes the Euclidean distance from point to the Line. The theory behind this algorithm is explained in: http://www.randygaul.net/2014/07/23/distance-point-to-line-segment/
     *
     * @param point Point to which the distance is computed.
     * @return A double value >= 0.0 which represents the distance to this Line.
     */
    [[nodiscard]] double distanceToLine(const Vector2 &point) const;

    /**
     * Projects the point onto this Line, i.e. find the location on this Line closest to that point.
     *
     * @param point Point that is projected on this Line.
     * @return The projection point. Note that this point is located on this Line and it is orthogonal to the given point.
     */
    [[nodiscard]] Vector2 project(const Vector2 &point) const;

    /**
     * Get the intersection point between two (infinite) Lines. In case there is no intersection (Lines are parallel and distinct), return std
     *
     * @param line The other (infinite) Line.
     * @return In case there is no intersection (i.e. the Lines are parallel and distinct), return std::nullopt. In case there is a single intersection, return that point (Lines
     * are non-parallel). In case there are multiple (infinitely many) intersections, return the intersection point closest to the origin (0,0).
     */
    [[nodiscard]] std::optional<Vector2> intersect(const Line &line) const;

    /**
     * Determine whether a point lies on this Line. This function is NOT protected against double/float rounding issues.
     *
     * @param point The point which might lie on the Line.
     * @return True if the given point lies on this Line, false otherwise.
     */
    [[nodiscard]] bool isOnLine(const Vector2 &point) const;

    /**
     * Get the intersection point between two infinite lines. No intersection point is returned in case the lines are equal, parallel or if one of the lines is undefined
     * (a line is undefined if the 2 given points, which define the line, are equal). See https://en.wikipedia.org/wiki/Line%E2%80%93line_intersection for more information.
     *
     * @param p1 An arbitrary point on the first line.
     * @param p2 An arbitrary other point on the first line.
     * @param q1 An arbitrary point on the second line.
     * @param q2 An arbitrary other point on the second line.
     * @return std::nullopt if the lines do not intersect, are equal or if one of the lines is undefined. Otherwise return the intersection point.
     */
    static std::optional<Vector2> intersect(const Vector2 p1, const Vector2 p2, const Vector2 q1, const Vector2 q2);

    /**
     * Get the relative position of pointOnLine on the given infinite line, i.e. compute t such that p1 + (p2 - p1) * t = pointOnLine. Make sure that the given
     * parameters p1 and p2 are different and that pointOnLine actually lies on that Line, otherwise the return value does not make sense.
     *
     * @param p1 An arbitrary point on the first line.
     * @param p2 An arbitrary other point on the first line (make sure that it is different than p1).
     * @param pointOnLine A point that is located on the given line.
     * @return The value t such that p1 + (p2 - p1) * t = pointOnLine
     */
    static float relativePosition(const Vector2 p1, const Vector2 p2, const Vector2 pointOnLine);

    /* 	 ______   _______  _______  ______     _______  _______  ______   _______
     *	(  __  \ (  ____ \(  ___  )(  __  \   (  ____ \(  ___  )(  __  \ (  ____ \
     *	| (  \  )| (    \/| (   ) || (  \  )  | (    \/| (   ) || (  \  )| (    \/
     *	| |   ) || (__    | (___) || |   ) |  | |      | |   | || |   ) || (__
     *	| |   | ||  __)   |  ___  || |   | |  | |      | |   | || |   | ||  __)
     *	| |   ) || (      | (   ) || |   ) |  | |      | |   | || |   ) || (
     *	| (__/  )| (____/\| )   ( || (__/  )  | (____/\| (___) || (__/  )| (____/\
     *	(______/ (_______/|/     \|(______/   (_______/(_______)(______/ (_______/
     *
     * The functions below are dead. Remove this tag if you use any of the functions and make sure to remove this tag at other places as well that will become alive by using any of
     *the function below. Do not read/document/redesign/analyse/test/optimize/etc. any of this code, because it is a waste of your time! This code was not removed or placed at
     *another branch, because other software developers are very attached to this code and are afraid that this code might be used at some day (but I think it won't be used at all
     *and should be removed).
     */

    /**
     * @brief Gets the intersection of the lines
     * See https://en.wikipedia.org/wiki/Line%E2%80%93line_intersection for help
     *
     * @param line LineSegment to get an intersection from
     * @return std::shared_ptr<Vector2> Vector representation of this intersection
     */
    // [[nodiscard]] std::optional<Vector2> intersects(const LineSegment &line) const;

    /**
     * @brief Checks whether \ref line intersects `this`
     *
     * @param line Line to check against
     * @return true True if \ref line intersects `this`
     * @return false False if \ref line does not intersect `this`
     */
    // [[nodiscard]] bool doesIntersect(const LineSegment &line) const;

    /**
     * @brief Checks whether 2 lines are parralel
     *
     * @param line Other line to check against
     * @return true True if this->slope() == line.slope()
     * @return false False if this->slope() != line.slope()
     */
    // [[nodiscard]] bool isParallel(const Line &line) const;

    /**
     * @brief Checks whether 2 lines are parralel
     *
     * @param line Other line to check against
     * @return true True if this->slope() == line.slope()
     * @return false False if this->slope() != line.slope()
     */
    // [[nodiscard]] bool isParallel(const LineSegment &line) const;

    /**
     * @brief Gets the intercept of this line
     * Literally:
     *      start.y - slope() * start.x;
     * @return double Intercept of this line
     */
    // [[nodiscard]] double intercept() const;

    /**
     * @brief Gets a pair of coefficients
     *
     * @return std::pair<double, double> Pair of doubles where .first == slope() and .second == intercept()
     */
    // [[nodiscard]] std::pair<double, double> coefficients() const;

    /**
     * @brief Gets the slope of this line
     *
     * @return double Slope of the line
     */
    // [[nodiscard]] double slope() const;

    /**
     * @brief Checks whether line is vertical
     *
     * @return true True if line is vertical, will be true if isPoint is true
     * @return false False if line is not vertical
     */
    // [[nodiscard]] bool isVertical() const;

    /**
     * @brief Checks whether \ref line intersects `this`
     *
     * @param line Line to check against
     * @return true if \ref line intersects `this`
     * @return false False if \ref line does not intersect `this`
     */
    // [[nodiscard]] bool doesIntersect(const Line &line) const;
};
}  // namespace rtt

#endif  // ROBOTEAM_UTILS_LINE_H
