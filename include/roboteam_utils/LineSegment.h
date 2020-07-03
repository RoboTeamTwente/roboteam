#ifndef ROBOTEAM_UTILS_LINESEGMENT_H
#define ROBOTEAM_UTILS_LINESEGMENT_H
#include "Definitions.h"
#include "Vector2.h"

namespace rtt {
class Line;

/**
 * The LineSegment class represents a finite length line that starts at a given point and ends at a given point. If you want to use an infinite length Line instead then use the
 * Line class instead.
 * @author Created by: Rolf van der Hulst <br>
 *         Documented and redesigned by: Haico Dorenbos
 * @since 2019-05-15
 */
class LineSegment {
    public:
        Vector2 start; // One of the endings of this LineSegment (which we define as the start of this LineSegment).
        Vector2 end; // Another ending of this LineSegment (which we define as the end of this LineSegment).

        /**
         * Creates a new LineSegment instance by giving 2 points which represents the locations between which the LineSegment is created.
         *
         * @param start One of the endings of this LineSegment.
         * @param end Another ending of this LineSegment (it is allowed to use the same location as start).
         */
        constexpr LineSegment(const Vector2 &start, const Vector2 &end) : start{start}, end{end} {};

        /**
         * Computes the length of this line, which is the Euclidean distance between both endings of this LineSegment.
         *
         * @return A double value >= 0.0 which represents the length.
         */
        [[nodiscard]] double length() const;

        /**
         * Computes the squared length of this line, which is the square of the Euclidean distance between both endings of this LineSegment.
         *
         * @return A double value >= 0.0 which represents the squared length.
         */
        [[nodiscard]] double length2() const;

        /**
         * Checks whether this LineSegment is also a point, i.e. check if both of the endings of this LineSegment are the same. This function is protected against double/float rounding
         * issues.
         *
         * @return True if the LineSegment is a point and false otherwise.
         */
        [[nodiscard]] bool isPoint() const;

        /**
         * Get the Euclidean distance from a point to this LineSegment.
         *
         * @param point Point to which the distance is measured.
         * @return A double value >= 0.0 which represents the distance to this LineSegment.
         */
        [[nodiscard]] double distanceToLine(const Vector2 &point) const;

        /**
         * Checks whether a point lies on this LineSegment. This function is NOT protected against double/float rounding issues.
         *
         * @param point Point which being checked if it lies at this LineSegment.
         * @return True if the point lies on this LineSegment and false otherwise.
         */
        [[nodiscard]] bool isOnLine(const Vector2 &point) const;

        /**
         * Projects the point onto this LineSegment, i.e. find the location on this LineSegment closest to that point.
         *
         * @param point Point that is projected on this LineSegment.
         * @return The projection point. Note that this point is located on this LineSegment, but it does NOT have to be orthogonal to the given point.
         */
        [[nodiscard]] Vector2 project(const Vector2 &point) const;

        /**
         * Get the intersection between two (finite) LineSegment. Note that this function knows how to deal with LineSegments that are points and how to deal with parallel
         * LineSegments. Moreover this function guarantees to return an intersection point if the LineSegments intersects. In case:
         * - There is no intersection then std:nullopt is returned.
         * - There is a single intersection point then that intersection point is returned.
         * - There are multiple intersection points (infinitely many) which happens when the LineSegments overlap then it prefers to returns 1: start, 2: end, 3: line.start of which
         *  the first most option is selected that lies on both LineSegments. Note that if none of these points lie on both LineSegments and if both the LineSegments are parallel then
         *  the LineSegments cannot intersect.
         *
         * @param line The other (finite) LineSegment.
         * @return std::nullopt if the LineSegments do not intersect. Otherwise return an intersection point.
         */
        [[nodiscard]] std::optional<Vector2> intersects(const LineSegment &line) const;

        /**
         * Check if two LineSegment intersect. Note that this function knows how to deal with LineSegments that are points and how to deal with parallel LineSegments.
         *
         * @param line The other (finite) LineSegment.
         * @return True if the LineSegments have any type of intersection (overlap is also considered as intersection) and false otherwise.
         */
        [[nodiscard]] bool doesIntersect(const LineSegment &line) const;

        /**
         * Return the intersection(s) of two LineSegments. This is function is similar to the intersects function in the sense that it knows how to deal with parallel LineSegments and
         * LineSegments that are actually points. But it is different from the intersects function in the sense that it can return up to 2 intersections. In case:
         * - The LineSegments do not intersect, it returns an empty list.
         * - The LineSegments do only intersect once, it returns a list with only that intersection point.
         * - The LineSegments intersect multiple times (infinitely often), then these intersections form a LineSegment L. In this case it returns the start and end points of
         *  LineSegment L.
         *
         * @param line The other (finite) LineSegment.
         * @return Returns a list of the intersections.
         */
        [[nodiscard]] std::vector<Vector2> multiIntersect(const LineSegment &line) const;

        /**
         * Destructor of the LineSegment.
         */
        virtual ~LineSegment() = default;

        /**
         * Checks if two LineSegments are actually the same, i.e. to check if the ending of the LineSegments are at the same locations. Note that the directions of the LineSegments
         * might be different in case of equality.
         * @param other The other LineSegment to which comparisons are made.
         * @return True if the LineSegments are the same, false otherwise.
         */
        [[nodiscard]] bool operator==(const LineSegment &other) const;

    private:
        /**
         * Check whether a given point lies on this LineSegment, given that this LineSegment has length > 0 and that the given point lies on the infinite Line expansion of this
         * LineSegment. (Note that you have to make sure that these assumptions holds, otherwise it does not make sense to call this function).
         * @param point The point which might lie at this LineSegment.
         * @return True if the point lies at this LineSegment, false otherwise.
         */
        [[nodiscard]] bool isOnFiniteLine(const Vector2 &point) const;

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
     * @brief Same as normal intersect, but always returns false if the lines are parallel.
     * intersection points of non-parallel lines are called non-simple (hence the name)
     *
     * @param line Line to check against
     * @return true True if \ref line intersects `this`
     * @return false False if \ref `line` does not intersect `this`
     */
    // [[nodiscard]] bool nonSimpleDoesIntersect(const LineSegment &line) const;

    /**
     * @brief Gets the slope of this line
     *
     * @return double Slope of the line
     */
    // [[nodiscard]] double slope() const;

    /**
     * @brief Gets the intercept of this line
     * Literally:
     *      start.y - slope() * start.x;
     * @return double Intercept of this line
     */
    // [[nodiscard]] double intercept() const;

    /**
     * @brief Gets the intersection of the lines
     * See https://en.wikipedia.org/wiki/Line%E2%80%93line_intersection for help
     *
     * @param line Line to get an intersection from
     * @return std::shared_ptr<Vector2> Vector representation of this intersection
     */
    // [[nodiscard]] std::optional<Vector2> intersects(const Line &line) const;

    /**
     * @brief Gets the shortest distance between any two points on both line segments
     * @param point Point to get distance to
     * @return distance between this line and the passed line.
     */
    // [[nodiscard]] double distanceToLine(const LineSegment &line) const;

    /**
     * @brief Checks whether \ref line intersects `this`
     *
     * @param line Line to check against
     * @return true if \ref line intersects `this`
     * @return false False if \ref line does not intersect `this`
     */
    // [[nodiscard]] bool doesIntersect(const Line &line) const;

    /**
     * @brief Checks whether 2 lines are parallel
     *
     * @param line Other line to check against
     * @return true True if this->slope() == line.slope()
     * @return false False if this->slope() != line.slope()
     */
    // [[nodiscard]] bool isParallel(const Line &line) const;

    /**
     * @brief Checks whether 2 lines are parallel
     *
     * @param line Other line to check against
     * @return true True if this->slope() == line.slope()
     * @return false False if this->slope() != line.slope()
     */
    // [[nodiscard]] bool isParallel(const LineSegment &line) const;

    /**
     * @brief Gets a pair of coefficients
     *
     * @return std::pair<double, double> Pair of doubles where .first == slope() and .second == intercept()
     */
    // [[nodiscard]] std::pair<double, double> coefficients() const;

    /**
     * @brief Checks whether line is vertical
     *
     * @return true True if line is vertical, will be true if isPoint is true
     * @return false False if line is not vertical
     */
    // [[nodiscard]] bool isVertical() const;

    /**
     * @brief Gets a vector representation of an intersection
     *
     * same as normal intersect, but always returns false if the lines are parallel
     * intersection points of non-parallel lines are called non-simple (hence the name)
     *
     * @param line Line to check against
     * @return std::shared_ptr<Vector2> Returns a shared_ptr to a Vector2 that represents the intersection
     */
    // [[nodiscard]] std::optional<Vector2> nonSimpleIntersects(const LineSegment &line) const;
};
}  // namespace rtt
#endif  // ROBOTEAM_UTILS_LINESEGMENT_H
