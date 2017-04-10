#pragma once

#include "Vector2.h"
#include <limits>
#include <utility>
#include <cmath>

namespace rtt {
    
constexpr double ARC_MAX = M_PIl * 2 - std::numeric_limits<float>::epsilon(); // not double: fmodl treats everything as floats...
    
class Arc {
    public:
    Vector2 center;
    double length;
    double width;
    double angleStart; // rad
    double angleEnd; // rad
    Arc(Vector2 center, double radius);
    Arc(Vector2 center, double radius, double angleStart, double angleEnd);
    Arc(Vector2 center, double length, double width, double angleStart, double angleEnd);
    
    bool isCircle() const;
    bool isPartialCircle() const;
    bool angleWithinArc(double angle) const;
    bool pointInArc(const Vector2& point) const;
    bool pointOnArc(const Vector2& point) const;
    
    std::pair<boost::optional<Vector2>, boost::optional<Vector2>> 
    intersectionWithLine(Vector2 lineStart, Vector2 lineEnd) const;
    
    private:
    static double normalize(double angle);
    boost::optional<Vector2> arcPointTowards(double angle) const;
    boost::optional<Vector2> checkAndDenormalize(Vector2 vec) const;
};
    
}