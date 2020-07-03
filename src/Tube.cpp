//
// Created by rolf on 24-02-20.
//

#include <Circle.h>
#include "Tube.h"
namespace rtt{
bool Tube::contains(const Vector2 &point) const {
    return lineSegment.distanceToLine(point)<=radius; // If the point is within radius of the line the tube contains the point.
}
bool Tube::isCircle() const {
    return lineSegment.isPoint();
}
Tube::Tube(const LineSegment& line, double radius) : lineSegment{line}, radius{radius} {
}
Tube::Tube(const Vector2& start, const Vector2& end, double radius) : lineSegment{LineSegment(start,end)},radius{radius} {
}
Tube::Tube() : lineSegment{LineSegment({0, 0}, {0, 0})},radius{1.0}{ // lines default construct to (0,0) to (0,0)
}
bool Tube::doesIntersectOrContain(const Circle &circle) const {
    return lineSegment.distanceToLine(circle.center)<=(radius+circle.radius);
}
Vector2 Tube::project(const Vector2 &point) {
    auto projectedPoint = lineSegment.project(point);
    return projectedPoint + (point - projectedPoint).stretchToLength(radius);
}

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

/*
bool Tube::doesIntersectOrContain(const LineSegment &line) const {
    return lineSegment.distanceToLine(line)<=radius; // If the distance between the two lines is smaller than radius the line intersects the tube
}
*/

}


