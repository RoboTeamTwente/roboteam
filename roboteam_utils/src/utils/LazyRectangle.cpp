#include "LazyRectangle.hpp"

#include <cassert>
#include <cmath>

#include "Line.h"
#include "LineSegment.h"
#include "Polygon.h"

constexpr const unsigned int INSIDE = 0x00;
constexpr const unsigned int LEFT = 0x01;
constexpr const unsigned int RIGHT = 0x02;
constexpr const unsigned int BOTTOM = 0x04;
constexpr const unsigned int TOP = 0x08;
namespace rtt {
    LazyRectangle::LazyRectangle(const Vector2 &corner, const Vector2 &oppositeCorner) : corner1{corner}, corner2{oppositeCorner} {}
    LazyRectangle::LazyRectangle(const Vector2 &bottomLeft, double x, double y) : corner1{bottomLeft}, corner2{Vector2(bottomLeft.x + x, bottomLeft.y + y)} {}
unsigned int LazyRectangle::CohenSutherlandCode(const Vector2 &point) const {
    double x = point.x;
    double y = point.y;
    unsigned int code = INSIDE;  // initialize code as if it's inside the clip window
    if (x < left()) {
        code |= LEFT;
    } else if (x > right()) {
        code |= RIGHT;
    }

    if (y < bottom()) {
        code |= BOTTOM;
    } else if (y > top()) {
        code |= TOP;
    }
    return code;
}
double LazyRectangle::left() const { return fmin(corner1.x, corner2.x); }
double LazyRectangle::right() const { return fmax(corner1.x, corner2.x); }
double LazyRectangle::bottom() const { return fmin(corner1.y, corner2.y); }
double LazyRectangle::top() const { return fmax(corner1.y, corner2.y); }
double LazyRectangle::width() const { return fabs(corner1.x - corner2.x); }
double LazyRectangle::height() const { return fabs(corner1.y - corner2.y); }

std::vector<Vector2> LazyRectangle::corners() const {
    std::vector<Vector2> corners = {bottomLeft(), topLeft(), topRight(), bottomRight()};
    return corners;
}

LineSegment LazyRectangle::topLine() const { return {topLeft(), topRight()}; }
LineSegment LazyRectangle::rightLine() const { return {topRight(), bottomRight()}; }
LineSegment LazyRectangle::bottomLine() const { return {bottomRight(), bottomLeft()}; }
LineSegment LazyRectangle::leftLine() const { return {bottomLeft(), topLeft()}; }

std::vector<LineSegment> LazyRectangle::lines() const {
    std::vector<Vector2> points = corners();
    std::vector<LineSegment> lines = {LineSegment(points[0], points[1]), LineSegment(points[1], points[2]), LineSegment(points[2], points[3]), LineSegment(points[3], points[0])};
    return lines;
}

Polygon LazyRectangle::asPolygon() const { return Polygon(corners()); }

Vector2 LazyRectangle::center() const { return (corner1 + corner2) * 0.5; }

Vector2 LazyRectangle::topLeft() const { return {this->left(), this->top()}; }
Vector2 LazyRectangle::topRight() const { return {this->right(), this->top()}; }
Vector2 LazyRectangle::bottomLeft() const { return {this->left(), this->bottom()}; }
Vector2 LazyRectangle::bottomRight() const { return {this->right(), this->bottom()}; }

// code borrowed from https://www.geeksforgeeks.org/line-clipping-set-1-cohen-sutherland-algorithm/
// and wikipedia. (I know it's way too long)
std::vector<Vector2> LazyRectangle::intersects(const LineSegment &line) const {
    unsigned int code0 = CohenSutherlandCode(line.start);
    unsigned int code1 = CohenSutherlandCode(line.end);
    std::vector<Vector2> intersections;
    bool accept = false;
    double x0 = line.start.x;
    double y0 = line.start.y;
    double x1 = line.end.x;
    double y1 = line.end.y;
    while (true) {
        if (!(code0 | code1)) {
            // bitwise OR is 0: both points inside window; trivially accept and exit loop
            accept = true;
            break;
        } else if (code0 & code1) {
            // bitwise AND is not 0: both points share an outside zone (LEFT, RIGHT, TOP,
            // or BOTTOM), so both must be outside window; exit loop (accept is false)
            break;
        } else {
            // failed both tests, so calculate the line segment to clip
            // from an outside point to an intersection with clip edge
            double x, y;

            // At least one endpoint is outside the clip rectangle; pick it.
            unsigned int codeOut = code0 ? code0 : code1;

            // Now find the intersection point;
            // use formulas:
            //   slope = (y1 - y0) / (x1 - x0)
            //   x = x0 + (1 / slope) * (ym - y0), where ym is ymin or ymax
            //   y = y0 + slope * (xm - x0), where xm is xmin or xmax
            // No need to worry about divide-by-zero because, in each case, the
            // outcode bit being tested guarantees the denominator is non-zero

            if (codeOut & TOP) {  // point is above the clip window
                x = x0 + (x1 - x0) * (top() - y0) / (y1 - y0);
                y = top();
            } else if (codeOut & BOTTOM) {  // point is below the clip window
                x = x0 + (x1 - x0) * (bottom() - y0) / (y1 - y0);
                y = bottom();
            } else if (codeOut & RIGHT) {  // point is to the right of clip window
                y = y0 + (y1 - y0) * (right() - x0) / (x1 - x0);
                x = right();
            } else{  // point is to the left of clip window
                assert(codeOut & LEFT);
                y = y0 + (y1 - y0) * (left() - x0) / (x1 - x0);
                x = left();
            }
            // Now we move outside point to intersection point to clip
            // and get ready for next pass.
            if (codeOut == code0) {
                x0 = x;
                y0 = y;
                Vector2 pt = Vector2(x0, y0);
                code0 = CohenSutherlandCode(pt);
                // Save point iff it is inside the Rect
                if (code0 == INSIDE) {
                    intersections.push_back(pt);
                }
            } else {
                x1 = x;
                y1 = y;
                Vector2 pt = Vector2(x1, y1);
                code1 = CohenSutherlandCode(pt);
                // Save point iff it is inside the Rect
                if (code1 == INSIDE) {
                    intersections.push_back(pt);
                }
            }
        }
    }
    return accept ? intersections : std::vector<Vector2>();
}
bool LazyRectangle::doesIntersect(const LineSegment &line) const { return !intersects(line).empty(); }

// include the boundary for this calculation!
bool LazyRectangle::contains(const Vector2 &point) const { return right() >= point.x && left() <= point.x && top() >= point.y && bottom() <= point.y; }

Vector2 LazyRectangle::project(const Vector2 &point) const {
    return {std::clamp(point.x, left(), right()),
            std::clamp(point.y, bottom(), top())};
}

std::ostream &LazyRectangle::write(std::ostream &os) const { return os << "Rect: " << corner1 << corner2; }

std::ostream &operator<<(std::ostream &out, const LazyRectangle &rect) { return rect.write(out); }

}  // namespace rtt
