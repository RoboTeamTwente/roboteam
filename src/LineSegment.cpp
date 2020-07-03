#include "../include/roboteam_utils/LineSegment.h"
#include "../include/roboteam_utils/Line.h"
#include "HalfLine.h"

namespace rtt {
double LineSegment::length() const { return (end - start).length(); }

double LineSegment::length2() const { return (end - start).length2(); }

bool LineSegment::isPoint() const { return start == end; }

bool LineSegment::doesIntersect(const LineSegment &line) const { return intersects(line).has_value(); }

std::optional<Vector2> LineSegment::intersects(const LineSegment &line) const {
    auto result = Line::intersect(start, end, line.start, line.end);
    if (result.has_value()) {
        float t = Line::relativePosition(start, end, result.value());
        float u = Line::relativePosition(line.start, line.end, result.value());
        if (t >= 0 & t <= 1 && u >= 0 && u <= 1) {
            return result;
        } else {
            return std::nullopt;
        }
    } else {
        /* The only possible cases are that one/both of the LineSegments are actually points, the LineSegments have a shared LineSegment part or the LineSegments are distinct and
         * parallel. */
        if (isPoint()) {
            return line.isOnLine(start) ? std::optional<Vector2>(start) : std::nullopt;
        } else if (line.isPoint()) {
            return isOnLine(line.start) ? std::optional<Vector2>(line.start) : std::nullopt;
        }

        // Check if both LineSegments are on the same infinite line.
        Line checkLine = Line(*this);
        if (!checkLine.isOnLine(line.start)) {
            return std::nullopt;
        }
        if (line.isOnFiniteLine(start)) {
            return start;
        } else if (line.isOnFiniteLine(end)) {
            return end;
        } else if (isOnFiniteLine(line.start)) {
            return line.start;
        } else {
            return std::nullopt;
        }
    }
}

double LineSegment::distanceToLine(const Vector2 &point) const { return (project(point) - point).length(); }

Vector2 LineSegment::project(const Vector2 &point) const {
    if (isPoint()) {
        return start;
    }
    Vector2 projection = Line(*this).project(point);
    double t = Line::relativePosition(start, end, projection);
    if (t < 0) {
        return start;
    } else if (t > 1) {
        return end;
    }
    return projection;
}

bool LineSegment::isOnLine(const Vector2 &point) const {
    if (isPoint()) {
        return (start == point || end == point);
    }
    if (Line(*this).isOnLine(point)) {
        float t = Line::relativePosition(start, end, point);
        return t >= 0 && t <= 1;
    }
    return false;
}

std::vector<Vector2> LineSegment::multiIntersect(const LineSegment &line) const {
    auto result = Line::intersect(start, end, line.start, line.end);
    if (result.has_value()) {
        float t = Line::relativePosition(start, end, result.value());
        float u = Line::relativePosition(line.start, line.end, result.value());
        if (t >= 0 & t <= 1 && u >= 0 && u <= 1) {
            return {result.value()};
        } else {
            return {};
        }
    } else {
        /* The only possible cases are that one/both of the LineSegments are actually points, the LineSegments have a shared LineSegment part or the LineSegments are distinct and
         * parallel. */
        if (isPoint()) {
            if (line.isOnLine(start)) {
                return {start};
            } else {
                return {};
            }
        } else if (line.isPoint()) {
            if (isOnLine(line.start)) {
                return {line.start};
            } else {
                return {};
            }
        }

        // Check if both LineSegments are on the same infinite line.
        Line checkLine = Line(*this);
        if (!checkLine.isOnLine(line.start)) {
            return {};
        }
        std::vector<Vector2> intersections = {};
        if (line.isOnFiniteLine(start)) {
            intersections.push_back(start);
        }
        if (line.isOnFiniteLine(end)) {
            intersections.push_back(end);
        }
        if (isOnFiniteLine(line.start)) {
            intersections.push_back(line.start);
        }
        if (isOnFiniteLine(line.end)) {
            intersections.push_back(line.end);
        }
        std::sort(intersections.begin(), intersections.end());
        intersections.erase(std::unique(intersections.begin(), intersections.end()), intersections.end());
        return intersections;
    }
}

bool LineSegment::isOnFiniteLine(const Vector2 &point) const {
    float t = Line::relativePosition(start, end, point);
    return t >= 0 && t <= 1;
}

bool LineSegment::operator==(const LineSegment &other) const { return ((start == other.start && end == other.end) || (start == other.end && end == other.start)); }

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

double LineSegment::slope() const {
    return (end.y - start.y)/(end.x - start.x);
}

double LineSegment::intercept() const {
    return start.y - this->slope()*start.x;
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

bool LineSegment::doesIntersect(const Line &line) const {
    if (intersects(line)) {
        return true;
    }
    return false;
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
bool LineSegment::isVertical() const {
    return (end.x == start.x) && (end.y != start.y);
}

std::pair<double, double> LineSegment::coefficients() const {
    return {slope(), intercept()};
}
*/
}  // namespace rtt
