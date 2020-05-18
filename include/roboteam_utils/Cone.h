/* 	 ______   _______  _______  ______     _______  _______  ______   _______
 *	(  __  \ (  ____ \(  ___  )(  __  \   (  ____ \(  ___  )(  __  \ (  ____ \
 *	| (  \  )| (    \/| (   ) || (  \  )  | (    \/| (   ) || (  \  )| (    \/
 *	| |   ) || (__    | (___) || |   ) |  | |      | |   | || |   ) || (__
 *	| |   | ||  __)   |  ___  || |   | |  | |      | |   | || |   | ||  __)
 *	| |   ) || (      | (   ) || |   ) |  | |      | |   | || |   ) || (
 *	| (__/  )| (____/\| )   ( || (__/  )  | (____/\| (___) || (__/  )| (____/\
 *	(______/ (_______/|/     \|(______/   (_______/(_______)(______/ (_______/
 *
 * This class contains only dead code. Remove this tag if you use this code and make sure to remove this tag at other places as well that will become alive by using this code.
 * Do not read/document/redesign/analyse/test/optimize/etc. any of this code, because it is a waste of your time! This code was not removed or placed at another branch, because
 * other software developers are very attached to this code and are afraid that this code might be used at some day (but I think it won't be used at all and should be removed).
 */

#pragma once

#include "Vector2.h"
#include <vector>
#include <string>
#include <iostream>

namespace rtt {

    /**
     * @brief Cone class used for arithmetic
     *
     * Unused as of right now, hence marked maybe unused
     */
    [[maybe_unused]] class Cone {
    public:
        /**
         * @brief Construct a new Cone object
         *
         * @param startPoint Start point of the cone
         * @param centerPoint Center point of the cone
         * @param distance Radius of the cone
         */
        Cone(Vector2 startPoint, Vector2 centerPoint, double distance);

        /**
         * @brief Construct a new Cone object
         *
         * @param startPoint Start point of the cone
         * @param sideA Side one of the cone
         * @param sideB Side two of the cone
         */
        Cone(Vector2 startPoint, Vector2 sideA, Vector2 sideB);

        /**
         * @brief Checks whether a point is within this cone
         *
         * @param point Point to be checked
         * @return true True if the point is within the cone
         * @return false If the point is not within the cone
         */
        bool IsWithinCone(Vector2 point) const noexcept;

        /**
         * @brief Checks whether a point is within this cone
         *
         * @param point Point to be checked
         * @param marginRadius Margin for validity
         * @return true True if the point is within margin of the cone
         * @return false False if the point is not within margin
         */
        bool IsWithinCone(Vector2 point, double marginRadius) const noexcept;

        /**
         * @brief DEPRECATED, ALWAYS RETURNS FALSE!
         *
         * @param point irrelevant
         * @return true Never
         * @return false Always
         */
        [[deprecated]] bool IsWithinField(Vector2 point) const noexcept;

        /**
         * @brief Gets closest point on the side of the current cone
         *
         * @param point Point to get the closest point to
         * @param closeTo
         * @return Vector2 Vector representation of the closest point
         */
        Vector2 ClosestPointOnSide(Vector2 point, Vector2 closeTo) const noexcept;

        /**
         * @brief Gets the second closest point on the side of the cone
         *
         * @param point Point to get the closest point to
         * @return Vector2 Vector representation of this point
         */
        Vector2 SecondClosestPointOnSide(Vector2 point) const noexcept;

        /**
         * @brief Gets the closest points on the sides of two cones
         *
         * @param otherCone Other cone to use in addition to `this`
         * @param point Point to get the closest point to
         * @param closeTo
         * @return Vector2 Vector representation of this closest point
         */
        Vector2 ClosestPointOnSideTwoCones(Cone otherCone, Vector2 point, Vector2 closeTo) const noexcept;

        /**
         * @brief Gets intersection between 2 lines
         *
         * @param line1Start Line 1 begin
         * @param line1Dir Line 1 end
         * @param line2Start Line 2 start
         * @param line2Dir Line 2 end
         * @return Vector2 Vector representation of this intersection
         */
        static Vector2
        LineIntersection(Vector2 line1Start, Vector2 line1Dir, Vector2 line2Start, Vector2 line2Dir) noexcept;

        /**
         * @brief Gets the intersections between this and another line
         *
         * @param lineStart Start of line to test against
         * @param lineEnd End of line to sest against
         * @param sideA Whether or not to use side A or B, if true side a is used
         * @return Vector2 Vector representation of this intersection
         */
        Vector2 LineIntersection(Vector2 lineStart, Vector2 lineEnd, bool sideA) const noexcept;

        /**
         * @brief Checks whether 2 cones overlap
         *
         * @param otherCone Other cone to test against this
         * @return true True if the cones overlap
         * @return false False if the cones don't overlap
         */
        bool DoConesOverlap(Cone otherCone) const noexcept;

        /**
         * @brief Merges two cones
         *
         * Does nothing if this and otherCone don't overlap
         *
         * @param otherCone Cone to merge with `this`
         * @return Cone A copy of the newly created cone
         */
        Cone MergeCones(Cone otherCone) const noexcept;

        /**
         * @brief Destroy the Cone object
         *
         */
        ~Cone() = default;

        /**
         * @brief Start of the current cone
         *
         */
        Vector2 start;

        /**
         * @brief Center of the current cone
         *
         */
        Vector2 center;

        /**
         * @brief Radius of the cone
         *
         */
        double radius;

        /**
         * @brief Angle of the sides of the cone
         *
         */
        double angle;

        /**
         * @brief Side 1 of the cone
         *
         */
        Vector2 side1;

        /**
         * @brief Side 2 of the cone
         *
         */
        Vector2 side2;
    private:
        // Draw drawer;
    };

} // rtt
