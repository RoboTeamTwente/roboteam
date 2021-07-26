#include "Vector2.h"

#include <cmath>

#include "Angle.h"
#include "Definitions.h"

namespace rtt {

Vector2::Vector2(rtt::Angle &angle, const double &length)
        :x{cos(angle) * length}, y{sin(angle) * length} { }

double Vector2::dot(const Vector2 &other) const {
    return this->x*other.x + this->y*other.y;
}

double Vector2::dist(const Vector2 &other) const {
    return sqrt(dist2(other));
}

double Vector2::dist2(const Vector2 &other) const {
    return (*this - other).length2();
}

Vector2 Vector2::scale(double scalar) const {
    return {x*scalar, y*scalar};
}

Vector2 Vector2::normalize() const {
    if (this->length() == 0.0)
        return {0.0, 0.0};

    double d = 1.0/length();
    return {x*d, y*d};
}

double Vector2::length() const {
    return sqrt(this->length2());
}

double Vector2::length2() const {
    return (x*x + y*y);
}

double Vector2::angle() const {
    return this->toAngle();
}

rtt::Angle Vector2::toAngle() const {
    return Angle(atan2(y, x));
}

Vector2 Vector2::lerp(const Vector2 &other, double factor) const {
    return this->scale(factor) + other.scale(1 - factor);
}

Vector2 Vector2::rotate(double radians) const {
    double c = cos(radians);
    double s = sin(radians);
    return Vector2(x*c - y*s, x*s + y*c);
}

Vector2 Vector2::rotateAroundPoint(double radians, const Vector2& pivot) const {
    double c = cos(radians);
    double s = sin(radians);

    // Calculate new position
    double rotatedX = (this->x - pivot.x) * c - (this->y - pivot.y) * s + pivot.x;
    double rotatedY = (this->x - pivot.x) * s + (this->y - pivot.y) * c + pivot.y;

    return Vector2(rotatedX, rotatedY);
}

Vector2 Vector2::project(const Vector2 &lineA, const Vector2 &lineB) const {
    Vector2 ab = lineB - lineA;
    Vector2 ap = *this - lineA;
    double t = ap.dot(ab)/ab.dot(ab);
    if (t < 0) {
        return lineA;
    }
    else if (t > 1) {
        return lineB;
    }
    return lineA + ab.scale(t);
}

Vector2 Vector2::project2(const Vector2 &ab) const {
    return ab.scale(
            this->dot(ab)/ab.dot(ab));
}

bool Vector2::isNotNaN() const {
    /**
     * NaN is defined as not being equal to itself
     */
    return x == x && y == y;
}

Vector2 Vector2::stretchToLength(double desiredLength) const {
    if (length() == 0.0) {
        return {desiredLength, 0};
    }
    double frac = desiredLength/length();
    return {x*frac, y*frac};
}

double Vector2::cross(const Vector2 &other) const {
    return this->x*other.y - this->y*other.x;
}

bool Vector2::operator==(const Vector2 &other) const {
    return fabs(this->x - other.x) < RTT_PRECISION_LIMIT && fabs(this->y - other.y) < RTT_PRECISION_LIMIT;
}

bool Vector2::operator!=(const Vector2 &other) const {
    return !(*this == other);
}

bool Vector2::operator<(const Vector2 &other) const {
    return this->length() < other.length();
}

Vector2 Vector2::operator+=(const Vector2 &other) {
    return {this->x += other.x, this->y += other.y};
}

Vector2 Vector2::operator+=(const double &scalar) {
    return {x += scalar, y += scalar};
}

Vector2 Vector2::operator-=(const Vector2 &other) {
    return {this->x -= other.x, this->y -= other.y};
}

Vector2 Vector2::operator-=(const double &scalar) {
    return {x -= scalar, y -= scalar};
}

Vector2 Vector2::operator*=(const Vector2 &other) {
    return {this->x *= other.x, this->y *= other.y};
}

Vector2 Vector2::operator*=(const double &scalar) {
    return {x *= scalar, y *= scalar};
}

Vector2 Vector2::operator/=(const Vector2 &other) {
    assert(!(other == Vector2()) && "Division by zero");
    return {this->x /= other.x, this->y /= other.y};
}

Vector2 Vector2::operator/=(const double &scalar) {
    return {x /= scalar, y /= scalar};
}

Vector2 Vector2::operator+(const Vector2 &other) const {
    return {x + other.x, y + other.y};
}

Vector2 Vector2::operator+(const double &scalar) const {
    return {x + scalar, y + scalar};
}

Vector2 Vector2::operator-(const Vector2 &other) const {
    return {this->x - other.x, this->y - other.y};
}

Vector2 Vector2::operator-(const double &scalar) const {
    return {x - scalar, y - scalar};
}

Vector2 Vector2::operator*(const Vector2 &other) const {
    return {this->x*other.x, this->y*other.y};
}

Vector2 Vector2::operator*(const double &scalar) const {
    return {x*scalar, y*scalar};
}

Vector2 Vector2::operator/(const Vector2 &other) const {
    return {this->x/other.x, this->y/other.y};
}

Vector2 Vector2::operator/(const double &scalar) const {
    return {x/scalar, y/scalar};
}

Vector2& Vector2::operator=(const proto::Vector2f &msg) {
    x = msg.x();
    y = msg.y();
    return *this;
}

Vector2::operator proto::Vector2f() const {
    proto::Vector2f msg;
    msg.set_x(x);
    msg.set_y(y);
    return msg;
}

std::ostream &Vector2::write(std::ostream &os) const {
    return os << "{ x = " << x << ", y = " << y << " }";
}

std::ostream &operator<<(std::ostream &os, rtt::Vector2 const &vec) {
    return vec.write(os);
}

}

