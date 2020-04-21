//
// Created by rolf on 15-5-19.
//

#ifndef ROBOTEAM_UTILS_LINESEGMENT_H
#define ROBOTEAM_UTILS_LINESEGMENT_H
#include "Vector2.h"
namespace rtt {
class Line;
/**
 * @brief LineSegment class
 *
 */
class LineSegment {
   public:
    /**
     * @brief Constructs a new LineSegment
     *
     */
    constexpr LineSegment() = default;

    /**
     * @brief Constructs a LineSegment
     *
     * @param _start Start of the Line
     * @param _end End of the Line
     *
     */
    constexpr LineSegment(const Vector2 &_start, const Vector2 &_end) : start{_start}, end{_end} {};
    /**
     * @brief Construct a LineSegment from a line.
     * @param line Line to construct LineSegment from
     */
    explicit LineSegment(const Line &line);
    /**
     * @brief Start of the line
     */
    Vector2 start;

    /**
     * @brief End of the line
     *
     */
    Vector2 end;

    /**
     * @brief Gets the length of the vector representation of this line
     * Literally:
     *      (end - start).length()
     * @return double Gets the length of the line
     */
    [[nodiscard]] double length() const;

    /**
     * @brief Gets the length of the vector representation of this line
     *
     * @return double Length of this Line
     */
    [[nodiscard]] double length2() const;

    /**
     * @brief Gets the slope of this line
     *
     * @return double Slope of the line
     */
    [[nodiscard]] double slope() const;

    /**
     * @brief Gets the intercept of this line
     * Literally:
     *      start.y - slope() * start.x;
     * @return double Intercept of this line
     */
    [[nodiscard]] double intercept() const;

    /**
     * @brief Gets the direction of the Line
     *
     * @return Vector2 Vector representation of the direction of this vector
     */
    [[nodiscard]] Vector2 direction() const;

    /**
     * @brief Gets a pair of coefficients
     *
     * @return std::pair<double, double> Pair of doubles where .first == slope() and .second == intercept()
     */
    [[nodiscard]] std::pair<double, double> coefficients() const;

    /**
     * @brief Checks whether line is vertical
     *
     * @return true True if line is vertical, will be true if isPoint is true
     * @return false False if line is not vertical
     */
    [[nodiscard]] bool isVertical() const;

    /**
     * @brief Checks whether 2 lines are parallel
     *
     * @param line Other line to check against
     * @return true True if this->slope() == line.slope()
     * @return false False if this->slope() != line.slope()
     */
    [[nodiscard]] bool isParallel(const Line &line) const;
    /**
     * @brief Checks whether 2 lines are parallel
     *
     * @param line Other line to check against
     * @return true True if this->slope() == line.slope()
     * @return false False if this->slope() != line.slope()
     */
    [[nodiscard]] bool isParallel(const LineSegment &line) const;
    /**
     * @brief Checks whether line is a single point
     *
     * @return true True if start == end
     * @return false False if start != end
     */
    [[nodiscard]] bool isPoint() const;

    /**
     * @brief Gets the distance from the line to a point
     *
     * @param point Point to get distance to
     * @return double distance between line and point
     */
    [[nodiscard]] double distanceToLine(const Vector2 &point) const;
    /**
     * @brief Gets the shortest distance between any two points on both line segments
     * @param point Point to get distance to
     * @return distance between this line and the passed line.
     */
    [[nodiscard]] double distanceToLine(const LineSegment &line) const;
    /**
     * @brief Checks whether a point is on the line
     *
     * @param point Point to check
     * @return true True if the point is on this line
     * @return false False if the point is not on this line
     */
    [[nodiscard]] bool isOnLine(const Vector2 &point) const;

    /**
     * @brief Gets the projection of \ref point to `this`
     *
     * @param point Point to project
     * @return Vector2 Vector representation of this projectoin
     */
    [[nodiscard]] Vector2 project(const Vector2 &point) const;

    /**
     * @brief Gets the intersection of the lines
     * See https://en.wikipedia.org/wiki/Line%E2%80%93line_intersection for help
     *
     * @param line Line to get an intersection from
     * @return std::shared_ptr<Vector2> Vector representation of this intersection
     */
    [[nodiscard]] std::optional<Vector2> intersects(const Line &line) const;

    /**
     * @brief Gets the intersection of the lines
     * See https://en.wikipedia.org/wiki/Line%E2%80%93line_intersection for help
     *
     * @param line LineSegment to get an intersection from
     * @return std::shared_ptr<Vector2> Vector representation of this intersection
     */
    [[nodiscard]] std::optional<Vector2> intersects(const LineSegment &line) const;

    /**
     * @brief Checks whether \ref line intersects `this`
     *
     * @param line Line to check against
     * @return true if \ref line intersects `this`
     * @return false False if \ref line does not intersect `this`
     */
    [[nodiscard]] bool doesIntersect(const Line &line) const;

    /**
     * @brief Checks whether \ref line intersects `this`
     *
     * @param line Line to check against
     * @return true True if \ref line intersects `this`
     * @return false False if \ref line does not intersect `this`
     */
    [[nodiscard]] bool doesIntersect(const LineSegment &line) const;

    /**
     * @brief Same as normal intersect, but always returns false if the lines are parallel.
     * intersection points of non-parallel lines are called non-simple (hence the name)
     *
     * @param line Line to check against
     * @return true True if \ref line intersects `this`
     * @return false False if \ref `line` does not intersect `this`
     */
    [[nodiscard]] bool nonSimpleDoesIntersect(const LineSegment &line) const;

    /**
     * @brief Gets a vector representation of an intersection
     *
     * same as normal intersect, but always returns false if the lines are parallel
     * intersection points of non-parallel lines are called non-simple (hence the name)
     *
     * @param line Line to check against
     * @return std::shared_ptr<Vector2> Returns a shared_ptr to a Vector2 that represents the intersection
     */
    [[nodiscard]] std::optional<Vector2> nonSimpleIntersects(const LineSegment &line) const;
    /**
     * @brief Return the intersection(s) of two LineSegments.
     *
     * This is the only correct implementation of intersects that works for every case, e.g. parallel lines and LineSegments that are actually points. In case:
     * - The LineSegments do not intersect, it returns an empty set.
     * - The LineSegments do only intersect once, it returns a set with the intersection location as Vector2.
     * - The intersection of these LineSegments is a LineSegment L, then it returns the start and end points of this LineSegment L.
     *
     * @param line Line to check against.
     * @return std::set<Vector2> Returns a set of the intersections (Vector2).
     */
    [[nodiscard]] std::vector<Vector2> correctIntersects(const LineSegment &line) const;

    /**
     * @brief Computes the shadow caused by an obstacle (LineSegment) and light source on this LineSegment.
     * The shadow is a part of this LineSegment. And something is considered to be in the shadow if the line between the source and the position on the line intersects with
     * the obstacle LineSegment (so if the source is on the obstacle LineSegment then everything is in the shadow).
     *
     * @param source The place where the 'light source' is located, i.d. the position from which the projected shadow is computed.
     * @param obstacle The obstacle LineSegment which blocks the 'lights' and therefore might cause a shadow on this LineSegment.
     * @param negligible_shadow_length If the length of the shadow is smaller than this value then there is no shadow (if not set then the negligible shadow length is 1e-6).
     * @return No Linesegment if there is no shadow or if the shadow is smaller than the negligible length. Otherwise return the Linesegment that represents the shadow on this
     * LineSegment.
     */
    [[nodiscard]] std::optional<LineSegment> shadow(const Vector2 &source, const LineSegment &obstacle, float negligible_shadow_length = 1e-6) const;

    /**
     * Checks if two LineSegments are actually the same, i.e. to check if the ending of the LineSegments are at the same locations. Note that the directionns of the lines might be
     * different in case of equality.
     * @param other The other LineSegment to which comparisons are made.
     * @return True if the LineSegments are the same, false otherwise.
     */
    [[nodiscard]] bool operator==(const LineSegment &other) const;

    /**
     * @brief Destroy the Line Segment object
     *
     */
    virtual ~LineSegment() = default;
};
}  // namespace rtt
#endif  // ROBOTEAM_UTILS_LINESEGMENT_H
