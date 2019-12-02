//
// Created by rolf on 15-5-19.
//

#ifndef ROBOTEAM_UTILS_LINEBASE_H
#define ROBOTEAM_UTILS_LINEBASE_H

#include "Vector2.h"
#include <memory>
#include <utility>

namespace rtt {
    class Line;
    class LineSegment;

    /**
     * @brief LineBase class, both LineSegment and Line inherit from this
     * 
     */
    class LineBase {
    public:
        /**
         * @brief Constructs a LineBase
         * 
         */
        constexpr LineBase()
                : start{ 0.0, 0.0 }, end{ 0.0, 0.0 } {};

        /**
         * @brief Constructs a LineBase
         * 
         * @param _start Start of the Line
         * @param _end End of the Line
         * 
         */
        constexpr LineBase(const Vector2 &_start, const Vector2 &_end)
                : start{ _start }, end{ _end } {};

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
         * @brief Gets the length of the vector representation of this line
         * Literally:
         *      (end - start).lenght()
         * @return double Gets the length of the line
         */
        [[nodiscard]] double length() const;

        /**
         * @brief Gets the length of the vector representation of this line
         * 
         * @return double Lenght of this Line
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
         * @brief Gets the directoin of the Line
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
        [[nodiscard]] bool isParallel(const LineBase &line) const;

        /**
         * @brief Checks whether line is a single point
         * 
         * @return true True if start == end
         * @return false Flase if start != end
         */
        [[nodiscard]] bool isPoint() const;

        /**
         * @brief Gets the distance from the line to a point
         * 
         * @param point Point to get distance to
         * @return double distance between line and point
         */
        [[nodiscard]] virtual double distanceToLine(const Vector2 &point) const = 0;

        /**
         * @brief Checks whether a point is on the line
         * 
         * @param point Point to check
         * @return true True if the point is on this line
         * @return false False if the point is not on this line
         */
        [[nodiscard]] virtual bool isOnLine(const Vector2 &point) const = 0;

        /**
         * @brief Gets the projection of \ref point to `this`
         * 
         * @param point Point to project
         * @return Vector2 Vector representation of this projectoin
         */
        [[nodiscard]] virtual Vector2 project(const Vector2 &point) const = 0;

        /**
         * @brief Gets the intersection of the lines
         * See https://en.wikipedia.org/wiki/Line%E2%80%93line_intersection for help
         * 
         * @param line Line to get an intersection from
         * @return std::shared_ptr<Vector2> Vector representation of this intersection
         */
        [[nodiscard]] virtual std::shared_ptr<Vector2> intersects(const Line &line) const = 0;

        /**
         * @brief Gets the intersection of the lines
         * See https://en.wikipedia.org/wiki/Line%E2%80%93line_intersection for help
         * 
         * @param line LineSegment to get an intersection from
         * @return std::shared_ptr<Vector2> Vector representation of this intersection
         */
        [[nodiscard]] virtual std::shared_ptr<Vector2> intersects(const LineSegment &line) const = 0;

        /**
         * @brief Checks whether \ref line intersects `this`
         * 
         * @param line Line to check against
         * @return true if \ref line intersects `this`
         * @return false False if \ref line does not intersect `this`
         */
        [[nodiscard]] virtual bool doesIntersect(const Line &line) const = 0;

        /**
         * @brief Checks whether \ref line intersects `this`
         * 
         * @param line Line to check against
         * @return true True if \ref line intersects `this`
         * @return false False if \ref line does not intersect `this`
         */
        [[nodiscard]] virtual bool doesIntersect(const LineSegment &line) const = 0;
    };
}

#endif //ROBOTEAM_UTILS_LINEBASE_H
