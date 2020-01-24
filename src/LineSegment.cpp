//
// Created by rolf on 15-5-19.
//

#include "../include/roboteam_utils/Line.h"
#include "../include/roboteam_utils/LineSegment.h"

namespace rtt {
LineSegment::LineSegment(const Line &line) {
    start = line.start;
    end = line.end;
}
double LineSegment::length() const {
    return (end - start).length();
}

double LineSegment::length2() const {
    return (end - start).length2();
}

double LineSegment::slope() const {
    return (end.y - start.y)/(end.x - start.x);
}

bool LineSegment::isVertical() const {
    return (end.x == start.x) && (end.y != start.y);
}

Vector2 LineSegment::direction() const {
    return Vector2(end - start);
}

double LineSegment::intercept() const {
    return start.y - this->slope()*start.x;
}

std::pair<double, double> LineSegment::coefficients() const {
    return {slope(), intercept()};
}

bool LineSegment::isPoint() const {
    return start == end;
}

bool LineSegment::isParallel(const LineSegment &line) const {
    // check if line is vertical, otherwise check the slope
    if (this->isVertical() || line.isVertical()) {
        return this->isVertical() && line.isVertical();
    }
    return this->slope() == line.slope();
}
bool LineSegment::isParallel(const Line &line) const {
    // check if line is vertical, otherwise check the slope
    if (this->isVertical() || line.isVertical()) {
        return this->isVertical() && line.isVertical();
    }
    return this->slope() == line.slope();
}
bool LineSegment::doesIntersect(const Line &line) const {
    if (intersects(line)) {
        return true;
    }
    return false;
}

//this is the algorithm from
// https://stackoverflow.com/questions/563198/how-do-you-detect-where-two-line-segments-intersect
//takes overlaps into account in contrast to the intersect() function
bool LineSegment::doesIntersect(const LineSegment &line) const {
    Vector2 p = start, q = line.start, r = direction(), s = line.direction();
    double denom = r.cross(s);
    double numer = (q - p).cross(r);
    if (denom == 0) {
        if (numer == 0) {
            // lines are colinear
            double t0 = (q - p).dot(r)/r.length2();
            double t1 = t0 + s.dot(r)/r.length2();
            if (t0 < 0) {
                return t1 >= 0;
            }
            else if (t0 > 1) {
                return t1 <= 1;
            }
            return true;
        }
    }
    else {
        double u = numer/denom;
        if (! (u < 0 || u > 1)) { //check if it's on the segment
            double t = (q - p).cross(s)/denom;
            return (! (t < 0 || t > 1));//check if it's on the segment
        }
    }
    return false;
}

bool LineSegment::nonSimpleDoesIntersect(const LineSegment &line) const {
    Vector2 p = start, q = line.start, r = direction(), s = line.direction();
    double denom = r.cross(s);
    double numer = (q - p).cross(r);
    if (denom != 0) {
        double u = numer/denom;
        if (! (u <= 0 || u >= 1)) {
            double t = (q - p).cross(s)/denom;
            if (! (t <= 0 || t >= 1)) {
                return true;
            }
        }
    }
    return false;
}

std::optional<Vector2> LineSegment::intersects(const Line &line) const {
    Vector2 A = start - end;
    Vector2 B = line.start - line.end;
    double denom = A.cross(B);
    if (denom != 0) {
        Vector2 C = start - line.start;
        double numer = C.cross(B);
        double t = numer/denom;
        if (! (t < 0 || t > 1)) {
            return start - A*t;
        }
    }
    return std::nullopt;

}

// only returns a vector if there is a point intersection. If a segment intersects does not return anything
std::optional<Vector2> LineSegment::intersects(const LineSegment &line) const {
    Vector2 A = start - end;
    Vector2 B = line.start - line.end;
    Vector2 C = start - line.start;
    double numer = C.cross(B);
    double denom = A.cross(B);
    if (denom != 0) {
        double t = numer/denom;
        double u = - A.cross(C)/denom;
        if (! (t < 0 || t > 1) && ! (u < 0 || u > 1)) {
            return start - A*t;
        }
    }
    else if (numer == 0) {
        double t0 = C.dot(A)/A.length2();
        double t1 = t0 + B.dot(A)/A.length2();
        //check the ends for point intersections
        if ((t0 == 0 && t1 < 0) || (t1 == 0 && t0 < 0)) {
            return start;
        }
        else if ((t0 == 1 && t1 > 1) || (t1 == 1 && t0 > 1)) {
            return end;
        }
    }
    return std::nullopt;
}

double LineSegment::distanceToLine(const Vector2 &point) const {
    return (this->project(point) - point).length();
}

//same principle but now we do not necessarily have an orthogonal vector but just pick the closest point on the segment
Vector2 LineSegment::project(const Vector2 &point) const {
    Vector2 AB = direction();
    Vector2 AP = point - start;
    double t = AP.dot(AB)/length2();
    if (t < 0) {
        return Vector2(start);
    }
    else if (t > 1) {
        return Vector2(end);
    }
    return Vector2(start + AB*t);
}

bool LineSegment::isOnLine(const Vector2 &point) const {
    Vector2 A = end - start;
    Vector2 B = point - start;
    double cross = A.cross(B);
    if (cross != 0) {
        return false;
    }
    //check if the point is in between the two points
    double dot = A.dot(B);
    if (dot < 0) {
        return false;
    }
    if (dot > A.dot(A)) {
        return false;
    }
    return true;
}

std::optional<Vector2> LineSegment::nonSimpleIntersects(const LineSegment &line) const {
    Vector2 A = start - end;
    Vector2 B = line.start - line.end;
    Vector2 C = start - line.start;
    double denom = A.cross(B);
    if (denom != 0) {
        double t = C.cross(B)/denom;
        double u = - A.cross(C)/denom;
        if (! (t <= 0 || t >= 1) && ! (u <= 0 || u >= 1)) {
            return start - A*t;
        }
    }
    return std::nullopt;
}

}
