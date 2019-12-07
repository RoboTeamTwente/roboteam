//
// Created by rolf on 18-4-19.
//

#ifndef ROBOTEAM_UTILS_LINE_H
#define ROBOTEAM_UTILS_LINE_H

#include "LineBase.h"

namespace rtt {
    /**
     * @brief Line class, inherits from Base
     * 
     */
    class Line : public LineBase {
    public:
        /**
         * @brief Construct a new Line object
         * 
         */
        Line() = default;

        /**
         * @brief Constructs a new Line object
         * 
         * Calls the LineBase constructor aswel
         * 
         * @param _start Start of the Line
         * @param _end End of the Line
         * 
         */
        constexpr Line(const Vector2 &_start, const Vector2 &_end)
                : LineBase(_start, _end) {};

        /**
         * @brief Gets the distance from \ref point to the line
         * 
         * @param point Point to get distance to
         * @return double Distance to line
         */
        [[nodiscard]] double distanceToLine(const Vector2 &point) const override;

        /**
         * @brief Checks whether a \ref point is on the line
         * 
         * @param point Point to check `this` against
         * @return true If \ref point is on `this`
         * @return false If \ref point is not on `this`
         */
        [[nodiscard]] bool isOnLine(const Vector2 &point) const override;

        /**
         * @brief Projects the point onto the line
         * 
         * @param point Point to get the projection to
         * @return Vector2 Vector projection from the line to the point
         */
        [[nodiscard]] Vector2 project(const Vector2 &point) const override;

        /**
         * @brief Gets the intersection of the lines
         * See https://en.wikipedia.org/wiki/Line%E2%80%93line_intersection for help
         * 
         * @param line Line to get an intersection from
         * @return std::shared_ptr<Vector2> Vector representation of this intersection
         */
        [[nodiscard]] std::shared_ptr<Vector2> intersects(const Line &line) const override;

        /**
         * @brief Gets the intersection of the lines
         * See https://en.wikipedia.org/wiki/Line%E2%80%93line_intersection for help
         * 
         * @param line LineSegment to get an intersection from
         * @return std::shared_ptr<Vector2> Vector representation of this intersection
         */
        [[nodiscard]] std::shared_ptr<Vector2> intersects(const LineSegment &line) const override;

        /**
         * @brief Checks whether \ref line intersects `this`
         * 
         * @param line Line to check against
         * @return true if \ref line intersects `this`
         * @return false False if \ref line does not intersect `this`
         */
        [[nodiscard]] bool doesIntersect(const Line &line) const override;

        /**
         * @brief Checks whether \ref line intersects `this`
         * 
         * @param line Line to check against
         * @return true True if \ref line intersects `this`
         * @return false False if \ref line does not intersect `this`
         */
        [[nodiscard]] bool doesIntersect(const LineSegment &line) const override;

    };

}

#endif //ROBOTEAM_UTILS_LINE_H
