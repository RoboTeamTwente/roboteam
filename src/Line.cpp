//
// Created by rolf on 18-4-19.
//

#include "../include/roboteam_utils/Line.h"
namespace rtt {

double LineBase::length() const {
    return (end - start).length();
}
double LineBase::length2() const {
    return (end - start).length2();
}
double LineBase::slope() const {
    return (end.y - start.y)/(end.x - start.x);
}
bool LineBase::isVertical() const {
    double sl=this->slope();
    return sl==std::numeric_limits<double>::infinity()||sl==-std::numeric_limits<double>::infinity();
}
Vector2 LineBase::direction() const {
    return Vector2(end-start);
}
double LineBase::intercept() const {
    return start.y - this->slope()*start.x;
}
std::pair<double, double> LineBase::coefficients() const {
    double sl = this->slope();
    double intercept = start.y - sl*start.x;
    return std::make_pair(sl, intercept);
}
bool LineBase::isPoint() const {
    return start == end;
}
bool LineBase::isParallel(const LineBase &line) const{
    // check if line is vertical, otherwise check the slope
    double sl=this->slope();
    if(sl==std::numeric_limits<double>::infinity()||sl==-std::numeric_limits<double>::infinity()){
        return line.isVertical();
    }
    else return sl==line.slope();
}
double Line::distanceToLine(const Vector2 &point) const {
    return (this->project(point)-point).length();
}
double LineSegment::distanceToLine(const Vector2 &point) const {
    return (this->project(point)-point).length();
}
///Computes the projection of point onto the line. This is identical to picking the closest point on the line
// if we project point P onto AB we can compute as A + dot(AP,AB) / dot(AB,AB) * AB
Vector2 Line::project(const Vector2 &point) const{
    Vector2 AB=direction();
    Vector2 AP=point-start;
    return Vector2(start+AB*AP.dot(AB)/length2());
}
//same principle but now we do not necessarily have an orthogonal vector but just pick the closest point on the segment
Vector2 LineSegment::project(const Vector2 &point) const{
    Vector2 AB=direction();
    Vector2 AP=point-start;
    double t=AP.dot(AB)/length2();
    if (t<0){
        return Vector2(start);
    }
    else if(t>1){
        return Vector2(end);
    }
    return Vector2(start+AB*t);
}

bool Line::isOnLine(const Vector2 &point) const {
    Vector2 A=end-start;
    Vector2 B=point-start;
    return A.cross(B)==0;
}
bool LineSegment::isOnLine(const Vector2 &point) const {
    Vector2 A=end-start;
    Vector2 B=point-start;
    double crpd=A.cross(B);
    if (crpd!=0){
        return false;
    }
    //check if the point is in between the two points
    double dot=A.dot(B);
    if (dot<0){
        return false;
    }
    if (dot>A.dot(A)){
        return false;
    }
    return true;
}
// see https://en.wikipedia.org/wiki/Line%E2%80%93line_intersection for help. These should be thoroughly tested
std::shared_ptr<Vector2> Line::intersects(const Line &line) const {
    Vector2 A=start-end;
    Vector2 B=line.start-line.end;
    double denom=A.cross(B);
    if (denom!=0){
        Vector2 C=start-line.start;
        double numer=-C.cross(B);
        double t=numer/denom;
        return std::make_shared<Vector2>(start+A*t);
    }
    return nullptr;
}
std::shared_ptr<Vector2> Line::intersects(const LineSegment &line) const {
    Vector2 A=start-end;
    Vector2 B=line.start-line.end;
    double denom=A.cross(B);
    if (denom!=0){
        Vector2 C=start-line.start;
        double numer=C.cross(A);
        double u=numer/denom;
        if (!(u<0||u>1)){
            return std::make_shared<Vector2>(line.start-B*u);
        }
    }
    return nullptr;
}

std::shared_ptr<Vector2> LineSegment::intersects(const Line &line) const {
    Vector2 A=start-end;
    Vector2 B=line.start-line.end;
    double denom=A.cross(B);
    if (denom!=0){
        Vector2 C=start-line.start;
        double numer=C.cross(B);
        double t=numer/denom;
        if (!(t<0||t>1)) {
            return std::make_shared<Vector2>(start-A*t);
        }
    }
    return nullptr;

}
// only returns a vector if there is a point intersection. If a segment intersects does not return anything
std::shared_ptr<Vector2> LineSegment::intersects(const LineSegment &line) const {
    Vector2 A=start-end;
    Vector2 B=line.start-line.end;
    Vector2 C=start-line.start;
    double numer=C.cross(B);
    double denom=A.cross(B);
    if (denom!=0){
        double t=numer/denom;
        double u=-A.cross(C)/denom;
        if (!(t<0||t>1)&&!(u<0||u>1)) {
            return std::make_shared<Vector2>(start-A*t);
        }
    }
    else if (numer==0){
        double t0=C.dot(A)/A.length2();
        double t1=t0+B.dot(A)/A.length2();
        //check the ends for point intersections
        if ((t0==0&&t1<0)||(t1==0&&t0<0)){
            return std::make_shared<Vector2>(Vector2(start));
        }
        else if((t0==1&&t1>1)||(t1==1&&t0>1)){
            return std::make_shared<Vector2>(Vector2(end));
        }
    }
    return nullptr;
}

bool Line::doesIntersect(const Line &line) const {
    return !this->isParallel(line);
}
bool Line::doesIntersect(const LineSegment &line) const {
    if (intersects(line)){
        return true;
    }
    return false;
}
bool LineSegment::doesIntersect(const Line &line) const {
    if (intersects(line)){
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
}
