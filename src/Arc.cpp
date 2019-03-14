#include "roboteam_utils/Arc.h"
#include "roboteam_utils/Math.h"
#include <iostream>

namespace rtt {
    
Arc::Arc() : Arc(Vector2(), 1.0) {}

Arc::Arc(Vector2 center, double radius) : Arc(center, radius, 0, ARC_MAX) {}

Arc::Arc(Vector2 center, double radius, double angleStart, double angleEnd)
    : Arc(center, radius, radius, angleStart, angleEnd) {}
 
Arc::Arc(Vector2 center, double length, double width, double angleStart, double angleEnd)
    : center(center), length(length), width(width), angleStart(normalize(angleStart)), angleEnd(normalize(angleEnd)) {

	if (fabs(angleEnd) < .0001) {
		angleEnd = ARC_MAX - .0001;
	}

}
 
bool Arc::isCircle() const {
    return length == width && angleStart == 0 && angleEnd == ARC_MAX;
}  

bool Arc::isPartialCircle() const {
    return length == width;
}
   
bool Arc::angleWithinArc(double angle) const {
    angle = normalize(angle);
    return angle >= angleStart && angle <= angleEnd;
}   
   
double Arc::normalize(double angle) {
    angle = fmodl(angle, M_PIl * 2);
    if (angle < 0) {
        angle = M_PIl * 2 + angle;
    }
    return angle;
}   

bool Arc::pointInArc(const Vector2& point) const {
    Vector2 normPoint = point - center;
    auto arcPoint = arcPointTowards(normPoint.angle());
    return arcPoint && normPoint.length() <= arcPoint->length();  
}

bool Arc::pointOnArc(const Vector2& point) const {
    Vector2 normPoint = point - center;
    auto arcPoint = arcPointTowards(normPoint.angle());
    return arcPoint && *arcPoint == normPoint;
}

boost::optional<Vector2> Arc::checkAndDenormalize(Vector2 vec) const {
    return normalize(angleEnd - angleStart) - normalize(vec.angle() - angleStart) >= 0 ?
            boost::optional<Vector2>(vec + center)
            :
            boost::none;
}

std::pair<boost::optional<Vector2>, boost::optional<Vector2>> 
Arc::intersectionWithLine(Vector2 lineStart, Vector2 lineEnd) const {
    // cannot use compound operator (-=) due to const-ness
    lineStart = lineStart - center;
    lineEnd = lineEnd - center;
    if (isPartialCircle()) {
        // simple case: http://mathworld.wolfram.com/Circle-LineIntersection.html
        Vector2 diff = lineEnd - lineStart;
        double dr = diff.length();
        double d = lineStart.x * lineEnd.y - lineEnd.x * lineStart.y;
        double discr = length * length * dr * dr - d * d;
        
        if (discr >= 0) {
            double sign = diff.y >= 0 ? 1 : -1;
            double x1 = (d * diff.y + sign * diff.x * sqrtl(discr)) / (dr * dr);
            double y1 = (-d * diff.x + fabsl(diff.y) * sqrtl(discr)) / (dr * dr);
            
            auto first = checkAndDenormalize(Vector2(x1, y1));
            
            boost::optional<Vector2> second;
            if (fabsl(discr < .0001)) {
                // single intersection
                second = boost::none;
            } else {
                // two intersections
                second = checkAndDenormalize(Vector2(
                            (d * diff.y - sign * diff.x * sqrtl(discr)) / (dr * dr),
                            (-d * diff.x - fabsl(diff.y) * sqrtl(discr)) / (dr * dr)
                         ));
            }
            return {first, second};
        } else {
            // no intersections
            return {boost::none, boost::none};
        }
        
    } else {
        throw std::invalid_argument("Ellipse-line intersection not yet implemented");
    }
}

boost::optional<Vector2> Arc::arcPointTowards(Vector2 point) const {
    return arcPointTowards((point - center).angle());
}

boost::optional<Vector2> Arc::arcPointTowards(double angle) const {
    // https://math.stackexchange.com/a/22068
    angle = normalize(angle);
    if (!angleWithinArc(angle)) {
        return boost::none;
    }
    
    // special cases
    // the 0.5 * pi and 1.5 * pi cases are important,
    // then tan(angle) is infinite and the normal calculation fails.
    if (angle < .000001) {
        return Vector2(length, 0);
    } else if (fabsl(angle - M_PI_2) < .000001) {
        return Vector2(0, width);
    } else if (fabsl(angle - M_PI) < .000001) {
        return Vector2(-length, 0);
    } else if (fabsl(angle - M_PI - M_PI_2) < .000001) {
        return Vector2(0, -width);
    }
    
    double tan2 = powl(tanl(angle), 2);
    double denom = sqrtl(width*width  + length * length / tan2);
    double sign = angle > M_PI_2l && angle < + M_PIl + M_PI_2l ? -1 : 1;
    Vector2 point(
        (sign * length * width) / (sqrtl(width*width  + length * length * tan2)),
        (sign * length * width) / (sqrtl(length*length + width*width / tan2))
    );
    return point;
}

}
