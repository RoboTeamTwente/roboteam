#include "LineSegment.h"

#include <Angle.h>

#include <cmath>
#include <algorithm>
#include <optional>

#include "HalfLine.h"
#include "Line.h"

#include "Print.h"

namespace rtt {
double LineSegment::length() const { return (end - start).length(); }

double LineSegment::length2() const { return (end - start).length2(); }

Vector2 LineSegment::center() const {
    return (this->start + this->end) / 2;
}

bool LineSegment::isPoint() const { return start == end; }

bool LineSegment::doesIntersect(const LineSegment &line) const { return intersects(line).has_value(); }

std::optional<Vector2> LineSegment::intersects(const LineSegment &line) const {
    auto result = Line::intersect(start, end, line.start, line.end);
    if (result.has_value()) {
        float t = Line::relativePosition(start, end, result.value());
        float u = Line::relativePosition(line.start, line.end, result.value());
        if (t >= 0 && t <= 1 && u >= 0 && u <= 1) {
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
        if (t >= 0 && t <= 1 && u >= 0 && u <= 1) {
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

Vector2 LineSegment::direction() const {
    return end-start;
}

void LineSegment::move(const Vector2 &by) {
    start+=by;
    end+=by;
}

void LineSegment::rotate (const Angle angle, const Vector2 rotationPoint){

    Vector2 midpoint = rotationPoint;

    Vector2 startNewMidpoint = Vector2 { start.x - midpoint.x, start.y - midpoint.y};
    Vector2 endNewMidpoint = Vector2 { end.x - midpoint.x, end.y - midpoint.y};

    Vector2 startRotated = Vector2 {(cos(angle)*startNewMidpoint.x - sin(angle)*startNewMidpoint.y), (sin(angle)*startNewMidpoint.x + cos(angle)*startNewMidpoint.y)};
    Vector2 endRotated = Vector2 {(cos(angle)*endNewMidpoint.x - sin(angle)*endNewMidpoint.y), (sin(angle)*endNewMidpoint.x + cos(angle)*endNewMidpoint.y) };

    startRotated.x += midpoint.x;
    startRotated.y += midpoint.y;

    endRotated.x += midpoint.x;
    endRotated.y += midpoint.y;

    start = startRotated;
    end = endRotated;
}

void LineSegment::resize(double toLength) {
    // If this line segment is on a point, resizing does not work
    if (this->isPoint()) {
        RTT_WARNING("Tried to resize LineSegment with length 0")
        return;
    }
    auto centerOfLine = this->center();

    // Calculate the vector from the center to the end of the line, and scale that for the new length
    Vector2 halfOfLineSegment = this->end - centerOfLine;
    double halfNewLength = toLength / 2;
    Vector2 newHalfOfLineSegment = halfOfLineSegment.stretchToLength(halfNewLength);
    // And update the end and start
    this->end = centerOfLine + newHalfOfLineSegment;
    this->start = centerOfLine - newHalfOfLineSegment;
}

std::optional<Vector2> LineSegment::firstIntersects(const LineSegment &line) const {
    // These copies will get optimized away but make it easier to read w.r.t the stackoverflow link
    Vector2 p = start;
    Vector2 r = direction();
    Vector2 q = line.start;
    Vector2 s = line.direction();

    double uDenom = r.cross(s);
    double uNumer = (q - p).cross(r);
    if (uDenom == 0) {
        if (uNumer == 0) {
            // Lines are colinear;
            // if the interval between t0 and t1 intersects [0,1] they lines overlap on this interval
            double t0 = (q - p).dot(r) / (r.dot(r));
            double t1 = t0 + s.dot(r) / (r.dot(r));
            if (t0 < 0) {
                if (t1 >= 0) {
                    // interval overlaps, we pick closest point which is from the start to the end of the line (p+0*r);
                    return p;
                }
            } else if (t0 > 1) {
                if (t1 <= 1) {
                    return p + r;  // Similar but now we have the end of the line
                }
            } else {
                // we return the point closest to the start of p
                return p + r * fmax(fmin(t0,t1),0);
            }
            return std::nullopt;  // there was no intersection with the interval [0,1] so the lineas are colinear but have no overlap
        } else {
            return std::nullopt;  // Lines are parallel and nonintersecting
        }
    } else {
        // if we find t and u such that p+tr=q+us for t and u between 0 and 1, we have found a valid intersection.
        double u = uNumer / uDenom;
        if (u >= 0 && u <= 1) {
            double t = (q - p).cross(s) / uDenom;
            if (t >= 0 && t <= 1) {
                return p + r * t;  // we found a intersection point!
            }
        }
    }
    return std::nullopt;
}
bool LineSegment::preciseDoesIntersect(const LineSegment&line) const{
    Vector2 p = start, q = line.start, r = direction(), s = line.direction();
    double denom = r.cross(s);
    double numer = (q - p).cross(r);
    if (denom == 0) {
        if (numer == 0) {
            // lines are colinear
            double t0 = (q - p).dot(r) / r.length2();
            double t1 = t0 + s.dot(r) / r.length2();
            if (t0 < 0) {
                return t1 >= 0;
            } else if (t0 > 1) {
                return t1 <= 1;
            }
            return true;
        }
    } else {
        double u = numer / denom;
        if (!(u < 0 || u > 1)) {  // check if it's on the segment
            double t = (q - p).cross(s) / denom;
            return (!(t < 0 || t > 1));  // check if it's on the segment
        }
    }
    return false;
}

std::optional<Vector2> LineSegment::getClosestPointToLine(const Line &other) const {
    // Get the intersection of this line and the other line
    auto intersection = rtt::Line::intersect(this->start, this->end, other.v1, other.v2);

    if (intersection.has_value()) {
        return this->project(intersection.value());
    }

    return std::nullopt;
}

}  // namespace rtt
