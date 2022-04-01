#include <roboteam_utils/FieldRectangle.hpp>

#include <cmath>

namespace rtt {

FieldRectangle::FieldRectangle() {
    this->top = 1.0;
    this->right = 1.0;
    this->bottom = 0.0;
    this->left = 0.0;
    this->width = this->right - this->left;
    this->height = this->top - this->bottom;
    this->center = Vector2((this->left + this->right) * 0.5, (this->bottom + this->top) * 0.5);
}

FieldRectangle::FieldRectangle(double top, double right, double bottom, double left) {
    this->top = top;
    this->right = right;
    this->bottom = bottom;
    this->left = left;
    this->width = right - left;
    this->height = top - bottom;
    this->center = Vector2((left + right) * 0.5, (bottom + top) * 0.5);
}

FieldRectangle::FieldRectangle(const Vector2& pointA, const Vector2& pointB) {
    this->top = std::fmax(pointA.y, pointB.y);
    this->right = std::fmax(pointA.x, pointB.x);
    this->bottom = std::fmin(pointA.y, pointB.y);
    this->left = std::fmin(pointA.x, pointB.x);
    this->width = this->right - this->left;
    this->height = this->top - this->bottom;
    this->center = (pointA + pointB) * 0.5;
}

bool FieldRectangle::operator==(const FieldRectangle& other) const {
    return this->top == other.top
        && this->right == other.right
        && this->bottom == other.bottom
        && this->left == other.left;
}

bool FieldRectangle::contains(const Vector2& point) const {
    return point.x > this->left && point.x < this->right
        && point.y > this->bottom && point.y < this->top;
}

bool FieldRectangle::contains(const Vector2& point, double margin) const {
    return point.x > (this->left - margin) && point.x < (this->right + margin)
        && point.y > (this->bottom - margin) && point.y < (this->top + margin);
}

Polygon FieldRectangle::asPolygon(double margin) const {
    Vector2 lowerLeft(this->left - margin, this->bottom - margin);

    double marginalizedWidth = this->width + 2 * margin;
    double marginalizedHeight = this->height + 2 * margin;

    return {lowerLeft, marginalizedWidth, marginalizedHeight};
}

double FieldRectangle::getTop() const {
    return this->top;
}
double FieldRectangle::getRight() const {
    return this->right;
}
double FieldRectangle::getBottom() const {
    return this->bottom;
}
double FieldRectangle::getLeft() const {
    return this->left;
}
double FieldRectangle::getWidth() const {
    return this->width;
}
double FieldRectangle::getHeight() const {
    return this->height;
}
Vector2 FieldRectangle::getCenter() const {
    return this->center;
}

Vector2 FieldRectangle::getTopLeft() const {
    return {this->left, this->top};
}

Vector2 FieldRectangle::getTopRight() const {
    return {this->right, this->top};
}

Vector2 FieldRectangle::getBottomLeft() const {
    return {this->left, this->bottom};
}

Vector2 FieldRectangle::getBottomRight() const {
    return {this->right, this->bottom};
}

LineSegment FieldRectangle::getTopSide() const {
    return {Vector2(this->left, this->top), Vector2(this->right, this->top)};
}

LineSegment FieldRectangle::getRightSide() const {
    return {Vector2(this->right, this->bottom), Vector2(this->right, this->top)};
}

LineSegment FieldRectangle::getBottomSide() const {
    return {Vector2(this->left, this->bottom), Vector2(this->right, this->bottom)};
}

LineSegment FieldRectangle::getLeftSide() const {
    return {Vector2(this->left, this->bottom), Vector2(this->left, this->top)};
}

} // namespace rtt