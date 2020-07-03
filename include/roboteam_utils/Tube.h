//
// Created by rolf on 24-02-20.
//

#ifndef ROBOTEAM_UTILS_TUBE_H
#define ROBOTEAM_UTILS_TUBE_H
#include "LineSegment.h"
namespace rtt{
class Circle;
/**
 * @author Rolf
 * @date 24-02-20
 * @brief A class that represents a tube, which can be seen as the points at a constant distance from a 2d line segment or a capped 2d cylinder
 */
class Tube {
    public:
        /**
         * @brief Constructs a default Tube, which is simply a unit circle.
         */
        Tube();
        /**
         * @brief Construct a tube around the line from start to end with radius
         * @param start Starting point
         * @param end  Ending point
         * @param radius Given radius
         */
        Tube(const Vector2& start, const Vector2& end, double radius);
        /**
         * @brief Construct a tube around the line with given radius
         * @param line Line to construct tube around
         * @param radius Given radius
         */
        Tube(const LineSegment& line, double radius);

        LineSegment lineSegment; //line defining the centre of the tube
        double radius; //constant radius of the tube
        /**
         * @brief Checks whether a point is inside or on the boundary of the tube
         * @param point to be considered
         * @return True if point is inside or on the boundary of the tube
         */
        [[nodiscard]] bool contains(const Vector2 &point) const;

        /**
         * @brief checks if the tube and circle overlap
         * @param circle Circle to check intersection with
         * @return True if tube and circle intersect
         */
         [[nodiscard]] bool doesIntersectOrContain(const Circle &circle) const;

        /**
         * @brief Checks if this tube is a circle (E.g. it's line is a point)
         * @return true if tube is a circle, false if not
         */
        [[nodiscard]] bool isCircle() const;

        /**
        * @brief Projects the given point onto this tube
        * @param point The point which to project onto this tube
        * @return The location where the point is projected onto the tube
        */
        Vector2 project(const Vector2& point);

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
         * @brief checks if the line intersects the tube or is contained within the tube
         * @param line to consider
         * @return True if the line intersects with the tube
         */
        // [[nodiscard]] bool doesIntersectOrContain(const LineSegment &line) const;
};
}


#endif //ROBOTEAM_UTILS_TUBE_H
