#include "roboteam_utils/Section.h"

namespace rtt {

Vector2 Section::intersection(const Section& other) const {
    double x1 = a.x,
           x2 = b.x,
           y1 = a.y,
           y2 = b.y,
           
           x3 = other.a.x,
           x4 = other.b.x,
           y3 = other.a.y,
           y4 = other.b.y;
           
    //https://en.wikipedia.org/wiki/Line%E2%80%93line_intersection#Given_two_points_on_each_line
    double divisor = (x1-x2) * (y3-y4) - (y1 - y2) * (x3 - x4);
    double t1 = (x1 * y2 - y1 * x2);
    double t2 = (x3 * y4 - y3 * x4);
    double x = (t1 * (x3 - x4) - (x1 - x2) * t2) / divisor;
    double y = (t1 * (y3 - y4) - (y1 - y2) * t2) / divisor;
    return Vector2(x, y);
}    
    
bool Section::pointOnLine(const Vector2& point) const {
    return point.dist(a) + point.dist(b) - length < .0001;
}    
    
}