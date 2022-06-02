#include <roboteam_utils/FieldRectangle.hpp>

#include <cmath>

namespace rtt {

FieldRectangle::FieldRectangle() {
    this->_top = 1.0;
    this->_right = 1.0;
    this->_bottom = 0.0;
    this->_left = 0.0;
    this->_width = this->_right - this->_left;
    this->_height = this->_top - this->_bottom;
    this->_center = Vector2((this->_left + this->_right) * 0.5, (this->_bottom + this->_top) * 0.5);
}

FieldRectangle::FieldRectangle(double top, double right, double bottom, double left) {
    this->_top = top;
    this->_right = right;
    this->_bottom = bottom;
    this->_left = left;
    this->_width = right - left;
    this->_height = top - bottom;
    this->_center = Vector2((left + right) * 0.5, (bottom + top) * 0.5);
}

FieldRectangle::FieldRectangle(const Vector2& pointA, const Vector2& pointB) {
    this->_top = std::fmax(pointA.y, pointB.y);
    this->_right = std::fmax(pointA.x, pointB.x);
    this->_bottom = std::fmin(pointA.y, pointB.y);
    this->_left = std::fmin(pointA.x, pointB.x);
    this->_width = this->_right - this->_left;
    this->_height = this->_top - this->_bottom;
    this->_center = (pointA + pointB) * 0.5;
}

bool FieldRectangle::operator==(const FieldRectangle& other) const {
    return this->_top == other._top
        && this->_right == other._right
        && this->_bottom == other._bottom
        && this->_left == other._left;
}

bool FieldRectangle::contains(const Vector2& point) const {
    return point.x > this->_left && point.x < this->_right
        && point.y > this->_bottom && point.y < this->_top;
}

bool FieldRectangle::contains(const Vector2& point, double margin) const {
    return point.x > (this->_left - margin) && point.x < (this->_right + margin)
        && point.y > (this->_bottom - margin) && point.y < (this->_top + margin);
}

Vector2 FieldRectangle::project(const Vector2& point) const {
    return Vector2(
        std::clamp(point.x, this->_left, this->_right),
        std::clamp(point.y, this->_bottom, this->_top)
    );
}

Polygon FieldRectangle::asPolygon(double margin) const {
    Vector2 lowerLeft(this->_left - margin, this->_bottom - margin);

    double marginalizedWidth = this->_width + 2 * margin;
    double marginalizedHeight = this->_height + 2 * margin;

    return {lowerLeft, marginalizedWidth, marginalizedHeight};
}

double FieldRectangle::top() const {
    return this->_top;
}
double FieldRectangle::right() const {
    return this->_right;
}
double FieldRectangle::bottom() const {
    return this->_bottom;
}
double FieldRectangle::left() const {
    return this->_left;
}
double FieldRectangle::width() const {
    return this->_width;
}
double FieldRectangle::height() const {
    return this->_height;
}
Vector2 FieldRectangle::center() const {
    return this->_center;
}

Vector2 FieldRectangle::topLeft() const {
    return {this->_left, this->_top};
}

Vector2 FieldRectangle::topRight() const {
    return {this->_right, this->_top};
}

Vector2 FieldRectangle::bottomLeft() const {
    return {this->_left, this->_bottom};
}

Vector2 FieldRectangle::bottomRight() const {
    return {this->_right, this->_bottom};
}

LineSegment FieldRectangle::topLine() const {
    return {Vector2(this->_left, this->_top), Vector2(this->_right, this->_top)};
}

LineSegment FieldRectangle::rightLine() const {
    return {Vector2(this->_right, this->_bottom), Vector2(this->_right, this->_top)};
}

LineSegment FieldRectangle::bottomLine() const {
    return {Vector2(this->_left, this->_bottom), Vector2(this->_right, this->_bottom)};
}

LineSegment FieldRectangle::leftLine() const {
    return {Vector2(this->_left, this->_bottom), Vector2(this->_left, this->_top)};
}

} // namespace rtt