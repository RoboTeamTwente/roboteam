//
// Created by rolf on 15-5-19.
//

#ifndef ROBOTEAM_UTILS_LINESEGMENT_H
#define ROBOTEAM_UTILS_LINESEGMENT_H
#include "LineBase.h"
namespace rtt {
    /**
     * @brief LineSegment class, inherits from LineBase
     * 
     */
    class LineSegment : public LineBase {
    public:

        /**
         * @brief Constructs a new LineSegment
         *
         */
        constexpr LineSegment() : LineBase(){};

        /**
         * @brief Constructs a LineBase
         *
         * @param _start Start of the Line
         * @param _end End of the Line
         *
         */
        constexpr LineSegment(const Vector2 &_start, const Vector2 &_end)
            : LineBase{ _start, _end } { };


        /**
         * @brief Gets the distance from the line to a point
         *
         * @param point Point to get distance to
         * @return double distance between line and point
         */
        [[nodiscard]] double distanceToLine(const Vector2 &point) const override;

        /**
         * @brief Checks whether a point is on the line
         *
         * @param point Point to check
         * @return true True if the point is on this line
         * @return false False if the point is not on this line
         */
        [[nodiscard]] bool isOnLine(const Vector2 &point) const override;

        /**
         * @brief Gets the projection of \ref point to `this`
         * 
         * @param point Point to project
         * @return Vector2 Vector representation of this projectoin
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

        /**
         * @brief Similar to doesIntersect however a different calculation
         *
         * same as normal intersect, but always returns false if the lines are parallel
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
        [[nodiscard]] std::shared_ptr<Vector2> nonSimpleIntersects(const LineSegment &line) const;

        /**
         * @brief Destroy the Line Segment object
         * 
         */
        virtual ~LineSegment() = default;

        };
}
#endif //ROBOTEAM_UTILS_LINESEGMENT_H
