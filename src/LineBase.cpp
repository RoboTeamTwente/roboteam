//
// Created by rolf on 15-5-19.
//

#include "../include/roboteam_utils/LineBase.h"
namespace rtt{
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
}
