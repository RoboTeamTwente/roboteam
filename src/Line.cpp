//
// Created by rolf on 18-4-19.
//

#include "../include/roboteam_utils/Line.h"
#include "../include/roboteam_utils/LineSegment.h"
namespace rtt {

double Line::distanceToLine(const Vector2 &point) const {
    return (this->project(point) - point).length();
}
///Computes the projection of point onto the line. This is identical to picking the closest point on the line
// if we project point P onto AB we can compute as A + dot(AP,AB) / dot(AB,AB) * AB
Vector2 Line::project(const Vector2 &point) const {
    Vector2 AB = direction();
    Vector2 AP = point - start;
    return Vector2(start + AB*AP.dot(AB)/length2());
}

bool Line::isOnLine(const Vector2 &point) const {
    Vector2 A = end - start;
    Vector2 B = point - start;
    return A.cross(B) == 0;
}

// see https://en.wikipedia.org/wiki/Line%E2%80%93line_intersection for help. These should be thoroughly tested
std::shared_ptr<Vector2> Line::intersects(const Line &line) const {
    Vector2 A = start - end;
    Vector2 B = line.start - line.end;
    double denom = A.cross(B);
    if (denom != 0) {
        Vector2 C = start - line.start;
        double numer = - C.cross(B);
        double t = numer/denom;
        return std::make_shared<Vector2>(start + A*t);
    }
    return nullptr;
}
std::shared_ptr<Vector2> Line::intersects(const LineSegment &line) const {
    Vector2 A = start - end;
    Vector2 B = line.start - line.end;
    double denom = A.cross(B);
    if (denom != 0) {
        Vector2 C = start - line.start;
        double numer = C.cross(A);
        double u = numer/denom;
        if (! (u < 0 || u > 1)) {
            return std::make_shared<Vector2>(line.start - B*u);
        }
    }
    return nullptr;
}

bool Line::doesIntersect(const Line &line) const {
    return ! this->isParallel(line);
}
bool Line::doesIntersect(const LineSegment &line) const {
    if (intersects(line)) {
        return true;
    }
    return false;
}
//this is the algorithm from
// https://stackoverflow.com/questions/563198/how-do-you-detect-where-two-line-segments-intersect
//takes overlaps into account in contrast to the intersect() function
bool LineSegment::doesIntersect(const LineSegment &line) const {
    Vector2 p=start,q=line.start,r=direction(),s=line.direction();
    double denom=r.cross(s);
    double numer=(q-p).cross(r);
    if (denom==0){
        if (numer==0){
            // lines are colinear
            double t0=(q-p).dot(r)/r.length2();
            double t1=t0+s.dot(r)/r.length2();
            if (t0<0){
                return t1>=0;
            }
            else if(t0>1){
                return t1<=1;
            }
            return true;
        }
    }
    else{
        double u= numer/denom;
        if (!(u<0||u>1)){
            double t=(q-p).cross(s)/denom;
            if (!(t<0||t>1)){
                return true;
            }
        }
    }
    return false;
}

// same as normal intersect, but always returns false if the lines are parallel
// intersection points of non-parallel lines are called non-simple (hence the name)
bool LineSegment::nonSimpleDoesIntersect(const LineSegment &line) const{
    Vector2 p=start,q=line.start,r=direction(),s=line.direction();
    double denom=r.cross(s);
    double numer=(q-p).cross(r);
    if (denom!=0){
        double u= numer/denom;
        if (!(u<=0||u>=1)){
            double t=(q-p).cross(s)/denom;
            if (!(t<=0||t>=1)){
                return true;
            }
        }
    }
    return false;
}
std::shared_ptr<Vector2> LineSegment::nonSimpleIntersects(const LineSegment &line) const {
    Vector2 A=start-end;
    Vector2 B=line.start-line.end;
    Vector2 C=start-line.start;
    double denom=A.cross(B);
    if (denom!=0){
        double t=C.cross(B)/denom;
        double u=-A.cross(C)/denom;
        if (!(t<=0||t>=1)&&!(u<=0||u>=1)) {
            return std::make_shared<Vector2>(start-A*t);
        }
    }
    return nullptr;
}

}
