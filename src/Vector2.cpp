#include "../include/roboteam_utils/Vector2.h"
#include "../include/roboteam_utils/Angle.h"

#include <cmath>

namespace rtt {

Vector2::Vector2(rtt::Angle &angle, const double &length)
        :epsilon(0.00001) {
    y = sin(angle.getAngle())*length;
    x = cos(angle.getAngle())*length;
}

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
    return this->toAngle().getAngle();
}

rtt::Angle Vector2::toAngle() const {
    if (this->length() < epsilon)
        return {};

    Angle a = Angle(atan2(y, x));
    return a;
}

Vector2 Vector2::lerp(const Vector2 &other, double factor) const {
    return this->scale(factor) + other.scale(1 - factor);
}

Vector2 Vector2::rotate(double radians) const {
    double c = cos(radians);
    double s = sin(radians);
    return Vector2(x*c - y*s, x*s + y*c);
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
    Vector2 ap = *this;
    double t = ap.dot(ab)/ab.dot(ab);
    return ab.scale(t);
}

bool Vector2::isNotNaN() const {
    return x == x && y == y; // NaN != NaN
}

Vector2 Vector2::closestPointOnVector(const Vector2 &startPoint, const Vector2 &point) const {
    Vector2 vectorToPoint = point - startPoint;
    Angle me = this->toAngle();
    Angle vtp = vectorToPoint.toAngle();
    Angle a = me - vtp;
    double angle = a.getAngle();
    double projectionLength = vectorToPoint.length()*cos(angle);

    Vector2 closestPoint;
    if (projectionLength > this->length()) {
        closestPoint = *this + startPoint;
    }
    else if (projectionLength < 0) {
        closestPoint = startPoint;
    }
    else {
        closestPoint = this->scale(projectionLength/this->length()) + startPoint;
    }
    return closestPoint;
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
    return fabs(this->x - other.x) < epsilon && fabs(this->y - other.y) < epsilon;
}

bool Vector2::operator!=(const Vector2 &other) const {
    return ! (*this == other);
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
    if (other == Vector2())
        throw std::invalid_argument("You mongol, stop dividing by zero");

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

void Vector2::operator=(const roboteam_proto::Vector2f &msg) {
    x = msg.x();
    y = msg.y();
}

Vector2::operator roboteam_proto::Vector2f() const {
    roboteam_proto::Vector2f msg;
    msg.set_x(x);
    msg.set_y(y);
    return msg;
}

std::ostream &Vector2::write(std::ostream &os) const {
    return os << "{ x = " << x << ", y = " << y << " }";
}

std::ostream &operator<<(std::ostream &os, const rtt::Vector2 vec) {
    return vec.write(os);
}

}

