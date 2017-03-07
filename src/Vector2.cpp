#include "roboteam_utils/Vector2.h"

#include <cmath>

namespace rtt {

    
double Vector2::dot(const Vector2& other) const {
    return x * other.x + y * other.y;
}

double Vector2::dist(const Vector2& other) const {
    return sqrt(dist2(other));
}

double Vector2::dist2(const Vector2& other) const {
    double dx = x - other.x;
    double dy = y - other.y;
    return dx*dx + dy*dy;
}

Vector2 Vector2::scale(double scalar) const {
    return Vector2(x*scalar, y*scalar);
}

Vector2 Vector2::normalize() const {
    if (length() == 0.0) {
        return Vector2();
    }
    double d = 1.0 / length();
    return Vector2(x*d, y*d);
}

double Vector2::length() const {
    return sqrt(x*x+y*y);
}

double Vector2::angle() const {
    return atan2(y,x);
}

Vector2 Vector2::lerp(const Vector2& other, double factor) const {
    return Vector2(x + (other.x - x) * factor, y + (other.y - y) * factor);
}

Vector2 Vector2::rotate(double radials) const {
    double c = cosl(radials);
    double s = sinl(radials);
    return Vector2(x * c - y * s, x * s + y * c);
}

Vector2 Vector2::project(const Vector2& line_a, const Vector2& line_b) const {
    Vector2 ab = line_b - line_a;
    Vector2 ap = *this - line_a;
    double t = ap.dot(ab) / ab.dot(ab);
    if (t < 0) {
        return line_a;
    } else if (t > 1) {
        return line_b;
    }
    return line_a + ab.scale(t);
}

bool Vector2::real() const {
    return x == x && y == y; // NaN check
}

Vector2 Vector2::closestPointOnVector(const Vector2& startPoint, const Vector2& point) const {
    Vector2 vectorToPoint = point - startPoint;
    double angle = this->angle() - vectorToPoint.angle();
    double projectionLength = vectorToPoint.length() * cos(angle);

    Vector2 closestPoint;
    if (projectionLength > this->length()) {
        closestPoint = *this + startPoint;
    } else if (projectionLength < 0) {
        closestPoint = startPoint;
    } else {
        closestPoint = this->scale(projectionLength / this->length()) + startPoint;
    }
    return closestPoint;
}

bool Vector2::operator==(const Vector2& other) const {
    return fabs(x-other.x) < 0.000001 && fabs(y-other.y) < 0.000001; 
}


bool Vector2::operator!=(const Vector2& other) const {
    return fabs(x-other.x) > 0.000001 || fabs(y-other.y) > 0.000001; 
}

Vector2 Vector2::operator+(const Vector2& other) const {
    return Vector2(x+other.x, y+other.y);
}

Vector2 Vector2::operator-(const Vector2& other) const {
    return Vector2(x-other.x, y-other.y);
}

Vector2 Vector2::operator*(const Vector2& other) const {
    return Vector2(x*other.x, y*other.y);
}
Vector2 Vector2::operator*(const double& other) const {
    return Vector2(x*other, y*other);
}
Vector2 Vector2::operator/(const double& other) const {
    return Vector2(x/other, y/other);
}

bool Vector2::operator<(const Vector2& other) const {
    return length() < other.length();
}

void Vector2::operator=(const roboteam_msgs::Vector2f& msg) {
    x = msg.x;
    y = msg.y;
}

Vector2::operator roboteam_msgs::Vector2f() const {
    roboteam_msgs::Vector2f msg;
    msg.x = x;
    msg.y = y;
    return msg;
}

std::ostream& Vector2::write(std::ostream& os) const {
    return os << "{ x = " << x << ", y = " << y << " }";
}

}

std::ostream& operator<<(std::ostream& os, const rtt::Vector2& vec) {
    return vec.write(os);
}
