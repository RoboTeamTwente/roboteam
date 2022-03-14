#include <FieldRectangle.hpp>

#include <cmath>

namespace rtt {

constexpr FieldRectangle::FieldRectangle() {
    this->top = 1.0;
    this->right = 1.0;
    this->bottom = 0.0;
    this->left = 0.0;
    this->width = this->right - this->left;
    this->height = this->top - this->bottom;
    this->center = Vector2((this->left + this->right) * 0.5, (this->bottom + this->top) * 0.5);
}

constexpr FieldRectangle::FieldRectangle(double top, double right, double bottom, double left) {
    this->top = top;
    this->right = right;
    this->bottom = bottom;
    this->left = left;
    this->width = right - left;
    this->height = top - bottom;
    this->center = Vector2((left + right) * 0.5, (bottom + top) * 0.5);
}

constexpr FieldRectangle::FieldRectangle(const Vector2& pointA, const Vector2& pointB) {
    this->top = std::fmax(pointA.y, pointB.y);
    this->right = std::fmax(pointA.x, pointB.x);
    this->bottom = std::fmin(pointA.y, pointB.y);
    this->left = std::fmin(pointA.x, pointB.x);
    this->width = right-left;
    this->height = top-bottom;
    this->center = (pointA + pointB) * 0.5;
}

bool FieldRectangle::operator==(const FieldRectangle& other) const {
    return top == other.top
        && right == other.right
        && bottom == other.bottom
        && left == other.left;
}

bool FieldRectangle::contains(const Vector2& point) const {
    return point.x > this->left && point.x < this->right
        && point.y > this->bottom && point.y < this->top;
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
const Vector2 FieldRectangle::getCenter() const {
    return this->center;
}

} // namespace rtt