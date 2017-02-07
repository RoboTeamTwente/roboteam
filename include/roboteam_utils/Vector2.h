#ifndef VECTOR2_H
#define VECTOR2_H

#include <iostream>

#include "roboteam_msgs/Vector2f.h"

namespace roboteam_utils
{

class Vector2
{
public:
	constexpr Vector2() : x(0), y(0) {}
	Vector2(double x, double y) : x(x), y(y) {}
	Vector2(const Vector2& copy) : x(copy.x), y(copy.y) {}
    Vector2(const roboteam_msgs::Vector2f& msg) : Vector2(msg.x, msg.y) {}

	double dot(const Vector2& other) const;
	double dist(const Vector2& other) const;
	double dist2(const Vector2& other) const;
	Vector2 scale(double scalar) const;
	Vector2 normalize() const;
	double length() const;
	double angle() const;
    Vector2 lerp(const Vector2& other, double factor) const;
    Vector2 rotate(double radials) const;
    Vector2 project(const Vector2& line_a, const Vector2& line_b) const;
    bool real() const;
    Vector2 closestPointOnVector(const Vector2& startPoint, const Vector2& point) const;

	bool operator==(const Vector2& other) const;
	bool operator!=(const Vector2& other) const;
    bool operator<(const Vector2& other) const;
	Vector2 operator+(const Vector2& other) const;
	Vector2 operator-(const Vector2& other) const;
	Vector2 operator*(const Vector2& other) const;
	Vector2 operator*(const double& other) const;
	Vector2 operator/(const double& other) const;
    void operator=(const roboteam_msgs::Vector2f& msg);
    operator roboteam_msgs::Vector2f() const;

    std::ostream& write(std::ostream& os) const;

	double x, y;
};

constexpr Vector2 ZERO_VECTOR = Vector2();

}

std::ostream& operator<<(std::ostream& os, const roboteam_utils::Vector2& vec);

#endif // VECTOR2_H
