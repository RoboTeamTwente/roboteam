#include "roboteam_utils/Section.h"

namespace rtt {

Vector2 Section::intersection(const Section& other) const {
    double x1 = a.x + 100,
           x2 = b.x + 100,
           y1 = a.y + 100,
           y2 = b.y + 100,
           
           x3 = other.a.x + 100,
           x4 = other.b.x + 100,
           y3 = other.a.y + 100,
           y4 = other.b.y + 100;
           
    //https://en.wikipedia.org/wiki/Line%E2%80%93line_intersection#Given_two_points_on_each_line
    double divisor = (x1-x2) * (y3-y4) - (y1 - y2) * (x3 - x4);
    if (fabs(divisor) < .0001) {
    	// Parallel or coincident
    	return {NAN, NAN};
    }
    double t1 = (x1 * y2 - y1 * x2);
    double t2 = (x3 * y4 - y3 * x4);
    double x = (t1 * (x3 - x4) - (x1 - x2) * t2) / divisor;
    double y = (t1 * (y3 - y4) - (y1 - y2) * t2) / divisor;
    return {x - 100, y - 100};
}

bool Section::pointOnLine(const Vector2& point) const {
    return point.dist(a) + point.dist(b) - length < .0001;
}    
    
bool Section::operator==(const Section& other) const {
    return a == other.a && b == other.b;
}

std::ostream& operator<<(std::ostream& stream, const Section& sec) {
    return stream << "Section[" << sec.a << ", " << sec.b << "]";
}

}
