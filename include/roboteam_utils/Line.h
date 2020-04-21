//
// Created by rolf on 18-4-19.
//

#ifndef ROBOTEAM_UTILS_LINE_H
#define ROBOTEAM_UTILS_LINE_H

#include "Vector2.h"

namespace rtt {
class LineSegment;
/**
 * @brief Line class, inherits from Base
 *
 */
class Line {
   public:
    /**
     * @brief Start of the line
     *
     */
    Vector2 start;
    /**
     * @brief End of the line
     *
     */
    Vector2 end;
    /**
     * @brief Construct a new Line object
     *
     */
    constexpr Line() = default;

    /**
     * @brief Constructs a new Line object
     *
     * @param _start Start of the Line
     * @param _end End of the Line
     *
     */
    constexpr Line(const Vector2 &_start, const Vector2 &_end) noexcept : start{_start}, end{_end} {};
    /**
     * @brief Constructs a new Line from a LineSegment.
     * @param other LineSegment to use for construction
     */
    explicit Line(const LineSegment &other) noexcept;
    /**
     * @brief Gets the length of the vector representation of this line.
     *        Note in this case it does not really make sense as this class represents lines of infinite length
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
     * @brief Checks whether 2 lines are parralel
     *
     * @param line Other line to check against
     * @return true True if this->slope() == line.slope()
     * @return false False if this->slope() != line.slope()
     */
    [[nodiscard]] bool isParallel(const Line &line) const;
    /**
     * @brief Checks whether 2 lines are parralel
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
     * @brief Gets the distance from \ref point to the line
     *
     * @param point Point to get distance to
     * @return double Distance to line
     */
    [[nodiscard]] double distanceToLine(const Vector2 &point) const;

    /**
     * @brief Checks whether a \ref point is on the line
     *
     * @param point Point to check `this` against
     * @return true If \ref point is on `this`
     * @return false If \ref point is not on `this`
     */
    [[nodiscard]] bool isOnLine(const Vector2 &point) const;

    /**
     * @brief Projects the point onto the line
     *
     * @param point Point to get the projection to
     * @return Vector2 Vector projection from the line to the point
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
     * @brief Compute the intersection of this, and the line given as parameter. The parameter line is treated as half-line, so for a line a + tb, with b the direction vector and
     * a the start parameter of this line, any point on the line where t<0 is not included in the calculation of the intersection.
     * @param line The line of which the positive direction intersection is computed.
     * @return No vector if the lines do not intersect in positive direction. Otherwise it returns a vector which represents the position of intersection.
     */
    [[nodiscard]] std::optional<Vector2> forwardIntersect(const Line &line) const;

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

   private:
    /**
     * Get the intersection point between this line and the given line as parameter. Moreover compute how far the given intersection is located relatively in the direction of the
     * start parameter of the line towards the end parameter of the line. Note if the line given as parameter is equal to this line then no intersection is returned.
     * @param line Line to check against.
     * @return std::nullopt if the lines do not intersect. Otherwise returns the intersection point and the relative extension of the given line to reach the intersection
     * point.
     */
    [[nodiscard]] std::optional<std::pair<Vector2, float>> generalIntersect(const Line &line) const;
};

}  // namespace rtt

#endif  // ROBOTEAM_UTILS_LINE_H
