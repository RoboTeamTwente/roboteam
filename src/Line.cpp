#include "../include/roboteam_utils/Line.h"
#include "../include/roboteam_utils/LineSegment.h"

namespace rtt {
Line::Line(const Vector2 &v1, const Vector2 &v2) {
    this->v1 = v1;
    this->v2 = v2;
    if (v1 == v2) {
        std::cerr << "Warning: you created an undefined line, because v1 == v2. Note that Lines have an infinite length. If you want to have a Line with finite length then use "
                     "the LineSegment class instead." << std::endl;
    }
}

Line::Line(const LineSegment &other) noexcept {
    v1 = other.start;
    v2 = other.end;
    if (v1 == v2) {
        std::cerr << "Warning: you created an undefined line, because v1 == v2. Note that Lines have an infinite length. If you want to have a Line with finite length then use "
                     "the LineSegment class instead." << std::endl;
    }
}

double Line::distanceToLine(const Vector2 &point) const { return (this->project(point) - point).length(); }

Vector2 Line::project(const Vector2 &point) const {
    Vector2 AB = v2 - v1;
    Vector2 AP = point - v1;
    return v1 + AB * (AP.dot(AB) / AB.length2());
}

bool Line::isOnLine(const Vector2 &point) const {
    Vector2 A = v2 - v1;
    Vector2 B = point - v1;
    return abs(A.cross(B)) < FLOAT_PRECISION;
}

std::optional<Vector2> Line::intersect(const Line &line) const {
    auto result = intersect(v1, v2, line.v1, line.v2);
    if (result.has_value()) {
        return result;
    } else if (line.isOnLine(v1)) {
        return project({0, 0});
    } else {
        return std::nullopt;
    }
}

std::optional<Vector2> Line::intersect(const Vector2 p1, const Vector2 p2, const Vector2 q1, const Vector2 q2) {
    Vector2 A = p1 - p2;
    Vector2 B = q1 - q2;
    double denom = A.cross(B);
    if (abs(denom) >= FLOAT_PRECISION) {
        Vector2 C = p1 - q1;
        double numer = C.cross(A);
        double u = numer / denom;
        return q1 - B * u;
    }
    return std::nullopt;
}

float Line::relativePosition(Vector2 p1, Vector2 p2, Vector2 pointOnLine) {
    float xDiff = p2.x - p1.x;
    if (xDiff == 0.0) {
        return (pointOnLine.y - p1.y) / (p2.y - p1.y);
    } else {
        return (pointOnLine.x - p1.x) / xDiff;
    }
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
std::optional<Vector2> Line::intersects(const LineSegment &line) const {
    auto result = intersect(start, end, line.start, line.end);
    if (result.has_value()) {
        float u = relativePosition(line.start, line.end, result.value());
        if (u >= 0 && u <= 1) return std::optional(result.value());
    }
    return std::nullopt;
}

bool Line::doesIntersect(const LineSegment &line) const {
    return intersects(line).has_value();
}

double Line::intercept() const {
    return start.y - this->slope()*start.x;
}

std::pair<double, double> Line::coefficients() const {
    return {slope(), intercept()};
}

bool Line::isParallel(const Line &line) const {
    // check if line is vertical, otherwise check the slope
    if (this->isVertical() || line.isVertical()) {
        return this->isVertical() && line.isVertical();
    }
    return this->slope() == line.slope();
}
bool Line::isParallel(const LineSegment &line) const {
    // check if line is vertical, otherwise check the slope
    if (this->isVertical() || line.isVertical()) {
        return this->isVertical() && line.isVertical();
    }
    return this->slope() == line.slope();
}

double Line::slope() const {
    return (end.y - start.y) / (end.x - start.x);
}

bool Line::isVertical() const {
    return (end.x == start.x) && (end.y != start.y);
}

bool Line::doesIntersect(const Line &line) const {
    return this->intersects(line).has_value();
}
*/
}  // namespace rtt
