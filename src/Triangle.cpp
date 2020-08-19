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

//
// Created by rolf on 24-01-20.
//

#include "Triangle.h"
#include "LineSegment.h"
#include "Line.h"

namespace rtt {
Triangle::Triangle(const Vector2 &point1, const Vector2 &point2, const Vector2 &point3)
        :corner1{point1}, corner2{point2}, corner3{point3} { }

// Efficient implementation, see this: https://stackoverflow.com/questions/2049582/how-to-determine-if-a-point-is-in-a-2d-triangle
bool Triangle::contains(const Vector2 &point) const {
    double as_x = point.x - corner1.x;
    double as_y = point.y - corner1.y;
    bool s_ab = (corner2.x - corner1.x)*as_y - (corner2.y - corner1.y)*as_x >= 0;
    if ((((corner3.x - corner1.x)*as_y - (corner3.y - corner1.y)*as_x) > 0) == s_ab) {
        return false;
    }
    return ((((corner3.x - corner2.x)*(point.y - corner2.y) - (corner3.y - corner2.y)*(point.x - corner2.x)) >= 0)
            == s_ab);
}
double Triangle::area() const {
    return abs(
            (corner1.x*(corner2.y - corner3.y) + corner2.x*(corner3.y - corner1.y) + corner3.x*(corner1.y - corner2.y))
                    *0.5);
}
std::vector<LineSegment> Triangle::lines() const {
    return std::vector<LineSegment>({LineSegment(corner1, corner2),
                                     LineSegment(corner2, corner3),
                                     LineSegment(corner3, corner1)});
}
std::vector<Vector2> Triangle::corners() const {
    return std::vector<Vector2>({corner1, corner2, corner3});
}
bool Triangle::doesIntersect(const Line &line) const {
    return line.intersects(LineSegment(corner1, corner2)) || line.intersects(LineSegment(corner2, corner3))
            || line.intersects(LineSegment(corner3, corner1));
}
bool Triangle::doesIntersect(const LineSegment &line) const {
    return line.intersects(LineSegment(corner1, corner2)) || line.intersects(LineSegment(corner2, corner3))
            || line.intersects(LineSegment(corner3, corner1));
}
std::vector<Vector2> Triangle::intersects(const LineSegment &line) const {
    std::vector<Vector2> intersections;
    for (const auto &triangleLine : lines()) {
        std::optional<Vector2> intersection = line.intersects(triangleLine);
        if (intersection) {
            intersections.push_back(*intersection);
        }
    }
    return intersections;
}
std::vector<Vector2> Triangle::intersects(const Line &line) const {
    std::vector<Vector2> intersections;
    for (const auto &triangleLine : lines()) {
        std::optional<Vector2> intersection = line.intersects(triangleLine);
        if (intersection) {
            intersections.push_back(*intersection);
        }
    }
    return intersections;
}
}
