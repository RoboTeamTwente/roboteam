//
// Created by rolf on 15-5-19.
//

#include "HalfLine.h"
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
    if (isPoint()) {
        return (start == point || end == point);
    }
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

std::vector<Vector2> LineSegment::correctIntersects(const LineSegment &line) const {
    Vector2 A = start - end;
    Vector2 B = line.start - line.end;
    Vector2 C = start - line.start;
    double denom = A.cross(B);
    if (denom == 0) {
        /* The only possible cases are that one/both of the LineSegments are actually points, the LineSegments have a shared LineSegment part or the LineSegments are distinct and
         * parallel. */
        std::vector<Vector2> intersections = {};
        if (line.isOnLine(start)) {
            intersections.push_back(start);
        }
        if (line.isOnLine(end)) {
            intersections.push_back(end);
        }
        if (isOnLine(line.start)) {
            intersections.push_back(line.start);
        }
        if (isOnLine(line.end)) {
            intersections.push_back(line.end);
        }
        std::sort(intersections.begin(), intersections.end());
        intersections.erase(std::unique(intersections.begin(), intersections.end()), intersections.end());
        return intersections;
    } else {
        // The lines are not parallel, so the only possible cases are that they either intersect or intersect when the lines are extended.
        double t = C.cross(B)/denom;
        double u = -A.cross(C)/denom;
        if (! (t < 0 || t > 1) && ! (u < 0 || u > 1)) {
            return {start - A*t};
        } else {
            return {};
        }
    }
}

// http://geomalgorithms.com/a07-_distance.html#dist3D_Segment_to_Segment
// for implementation hints. This is complicated unfortunately.
double LineSegment::distanceToLine(const LineSegment &line) const {
    Vector2 u = end - start;
    Vector2 v = line.end - line.start;
    Vector2 w = start - line.start;
    double uu = u.dot(u);
    double uv = u.dot(v);
    double vv = v.dot(v);
    double uw = u.dot(w);
    double vw = v.dot(w);
    double D = uu*vv-uv*uv; // Always >=0 by definition

    double sN, sD = D;       // sc = sN / sD, default sD = D >= 0
    double tN, tD = D;       // tc = tN / tD, default tD = D >= 0

    // compute the line parameters of the two closest points
    if (D == 0) { // the lines are parallel
        sN = 0.0;         // force using point start on this segment
        sD = 1.0;         // to prevent possible division by 0.0 later
        tN = vw;
        tD = vv;
    }
    else {                 // get the closest points on the infinite lines
        sN = (uv*vw - vv*uw);
        tN = (uu*vw - uv*uw);
        if (sN < 0.0) {        // sc < 0 => the s=0 edge is visible
            sN = 0.0;
            tN = vw;
            tD = vv;
        }
        else if (sN > sD) {  // sc > 1  => the s=1 edge is visible
            sN = sD;
            tN = vw + uv;
            tD = vv;
        }
    }

    if (tN < 0.0) {            // tc < 0 => the t=0 edge is visible
        tN = 0.0;
        // recompute sc for this edge
        if (-uw < 0.0)
            sN = 0.0;
        else if (-uw > uu)
            sN = sD;
        else {
            sN = -uw;
            sD = uu;
        }
    }
    else if (tN > tD) {      // tc > 1  => the t=1 edge is visible
        tN = tD;
        // recompute sc for this edge
        if ((-uw + uv) < 0.0)
            sN = 0;
        else if ((-uw + uv) > uu)
            sN = sD;
        else {
            sN = (-uw +  uv);
            sD = uu;
        }
    }
    double sc = sN == 0.0 ? 0.0 : sN / sD;
    double tc = tN == 0.0 ? 0.0 : tN / tD;
    Vector2 diff = w + (u * sc) - (v * tc);
    return diff.length();
}

std::optional<LineSegment> LineSegment::shadow(const Vector2 &source, const LineSegment &obstacle, float negligible_shadow_length) const {
    if (obstacle.isOnLine(source)) {
        // If the source is on the obstacle line then the entire line is in the shadow, because every line from the source intersects with the obstacle line.
        return length() > negligible_shadow_length ? std::optional(*this) : std::nullopt;
    } else if (isPoint()) {
        /* If the projection LineSegment is a point then there is no shadow on this LineSegment. This case is needed, because a point cannot be changed into an infinite line,
         * since the direction of a point is not known. */
        return std::nullopt;
    }
    std::optional<Vector2> firstIntersect = HalfLine(source, obstacle.start).intersect(Line(*this));
    std::optional<Vector2> secondIntersect = HalfLine(source, obstacle.end).intersect(Line(*this));
    if (!firstIntersect.has_value() && !secondIntersect.has_value()) {
        // If both lines from the sources to the start and end of the obstacle do not intersect with the infinite line expansion of this projection line then there is no shadow.
        return std::nullopt;
    }
    LineSegment shadow;
    if (firstIntersect.has_value() && secondIntersect.has_value()) {
        /* If they do intersect then we can compute the shadow by projecting points in the infinite expansion of the LineSegments to either endings of the LineSegment (note
         * projection does not change the intersection location if the location is already at the LineSegment). */
        shadow = LineSegment(project(firstIntersect.value()), project(secondIntersect.value()));
    } else {
        /* If there is only one intersection then the shadow is unbounded in one direction. By only checking if the ending furthest away from the intersection point is visible you
        know for all cases (both for the cases where the intersection point is outside the LineSegment and at the LineSegment) where the shadow is. Moreover you do this quite
        efficiently, because you only have to check for one LienSegment whether it intersects with the obstacle. */
        Vector2 onlyIntersection = project(firstIntersect.has_value() ? firstIntersect.value() : secondIntersect.value());
        double distanceIntersectionToStart = (start - onlyIntersection).length();
        double distanceIntersectionToEnd = (end - onlyIntersection).length();
        Vector2 closest = distanceIntersectionToStart < distanceIntersectionToEnd ? start : end;
        Vector2 furthest = distanceIntersectionToStart < distanceIntersectionToEnd ? end : start;
        bool furthestInShadow = LineSegment(source, furthest).doesIntersect(obstacle);
        shadow = furthestInShadow ? LineSegment(onlyIntersection, furthest) : LineSegment(onlyIntersection, closest);
    }
    return shadow.length() > negligible_shadow_length ? std::optional(shadow) : std::nullopt;
}

bool LineSegment::operator==(const LineSegment &other) const {
    return ((start == other.start && end == other.end) || (start == other.end && end == other.start));
}
}
