#include <cmath>
#include <roboteam_utils/FastRectangle.hpp>

namespace rtt {

FastRectangle::FastRectangle() {
    this->_top = 1.0;
    this->_right = 1.0;
    this->_bottom = 0.0;
    this->_left = 0.0;
    this->_width = this->_right - this->_left;
    this->_height = this->_top - this->_bottom;
    this->_center = Vector2((this->_left + this->_right) * 0.5, (this->_bottom + this->_top) * 0.5);
}

FastRectangle::FastRectangle(double top, double right, double bottom, double left) {
    this->_top = top;
    this->_right = right;
    this->_bottom = bottom;
    this->_left = left;
    this->_width = right - left;
    this->_height = top - bottom;
    this->_center = Vector2((left + right) * 0.5, (bottom + top) * 0.5);
}

FastRectangle::FastRectangle(const Vector2& pointA, const Vector2& pointB) {
    this->_top = std::fmax(pointA.y, pointB.y);
    this->_right = std::fmax(pointA.x, pointB.x);
    this->_bottom = std::fmin(pointA.y, pointB.y);
    this->_left = std::fmin(pointA.x, pointB.x);
    this->_width = this->_right - this->_left;
    this->_height = this->_top - this->_bottom;
    this->_center = (pointA + pointB) * 0.5;
}

bool FastRectangle::operator==(const FastRectangle& other) const {
    return this->_top == other._top && this->_right == other._right && this->_bottom == other._bottom && this->_left == other._left;
}

bool FastRectangle::contains(const Vector2& point) const { return point.x > this->_left && point.x < this->_right && point.y > this->_bottom && point.y < this->_top; }

bool FastRectangle::contains(const Vector2& point, double margin) const {
    return point.x > (this->_left - margin) && point.x < (this->_right + margin) && point.y > (this->_bottom - margin) && point.y < (this->_top + margin);
}

Vector2 FastRectangle::project(const Vector2& point) const { return Vector2(std::clamp(point.x, this->_left, this->_right), std::clamp(point.y, this->_bottom, this->_top)); }

Polygon FastRectangle::asPolygon(double margin) const {
    Vector2 lowerLeft(this->_left - margin, this->_bottom - margin);

    double marginalizedWidth = this->_width + 2 * margin;
    double marginalizedHeight = this->_height + 2 * margin;

    return {lowerLeft, marginalizedWidth, marginalizedHeight};
}

double FastRectangle::top() const { return this->_top; }
double FastRectangle::right() const { return this->_right; }
double FastRectangle::bottom() const { return this->_bottom; }
double FastRectangle::left() const { return this->_left; }
double FastRectangle::width() const { return this->_width; }
double FastRectangle::height() const { return this->_height; }
Vector2 FastRectangle::center() const { return this->_center; }

Vector2 FastRectangle::topLeft() const { return {this->_left, this->_top}; }

Vector2 FastRectangle::topRight() const { return {this->_right, this->_top}; }

Vector2 FastRectangle::bottomLeft() const { return {this->_left, this->_bottom}; }

Vector2 FastRectangle::bottomRight() const { return {this->_right, this->_bottom}; }

LineSegment FastRectangle::topLine() const { return {this->topLeft(), this->topRight()}; }

LineSegment FastRectangle::rightLine() const { return {this->topRight(), this->bottomRight()}; }

LineSegment FastRectangle::bottomLine() const { return {this->bottomRight(), this->bottomLeft()}; }

LineSegment FastRectangle::leftLine() const { return {this->bottomLeft(), this->topLeft()}; }

}  // namespace rtt