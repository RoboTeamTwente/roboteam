#include "roboteam_utils/Vector2.h"

#include <cmath>

namespace roboteam_utils {

Vector2::~Vector2(){} // NOP

double Vector2::dot(const Vector2& other) {
    return x * other.x + y * other.y;
}

double Vector2::dist(const Vector2& other) {
    return sqrt(dist2(other));
}

double Vector2::dist2(const Vector2& other) {
    double dx = x - other.x;
    double dy = y - other.y;
    return dx*dx + dy*dy;
}

Vector2 Vector2::scale(double scalar) {
    return Vector2(x*scalar, y*scalar);
}

Vector2 Vector2::normalize() {
    if (length() == 0.0) {
        return Vector2();
    }
    double d = 1.0 / length();
    return Vector2(x*d, y*d);
}

double Vector2::length() {
    return sqrt(x*x+y*y);
}

bool Vector2::operator==(const Vector2& other) {
    return fabs(x-other.x) < 0.000001 && fabs(y-other.y) < 0.000001; 
}


bool Vector2::operator!=(const Vector2& other) {
    return fabs(x-other.x) > 0.000001 || fabs(y-other.y) > 0.000001; 
}

Vector2 Vector2::operator+(const Vector2& other) {
    return Vector2(x+other.x, y+other.y);
}

Vector2 Vector2::operator-(const Vector2& other) {
    return Vector2(x-other.x, y-other.y);
}

Vector2 Vector2::operator*(const Vector2& other) {
    return Vector2(x*other.x, y*other.y);
}
Vector2 Vector2::operator*(const double& other) {
    return Vector2(x*other, y*other);
}
Vector2 Vector2::operator/(const double& other) {
    return Vector2(x/other, y/other);
}
}
