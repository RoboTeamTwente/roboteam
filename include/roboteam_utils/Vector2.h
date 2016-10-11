#ifndef VECTOR2_H
#define VECTOR2_H

namespace roboteam_utils
{

class Vector2
{
public:
	Vector2() : x(0), y(0) {}
	Vector2(double x, double y) : x(x), y(y) {}
	Vector2(const Vector2& copy) : x(copy.x), y(copy.y) {}
	~Vector2();
	double dot(const Vector2& other) const;
	double dist(const Vector2& other) const;
	double dist2(const Vector2& other) const;
	Vector2 scale(double scalar) const;
	Vector2 normalize() const;
	double length() const;
	double angle() const;
    Vector2 lerp(const Vector2& other, double factor) const;
	bool operator==(const Vector2& other) const;
	bool operator!=(const Vector2& other) const;
    bool operator<(const Vector2& other) const;
	Vector2 operator+(const Vector2& other) const;
	Vector2 operator-(const Vector2& other) const;
	Vector2 operator*(const Vector2& other) const;
	Vector2 operator*(const double& other) const;
	Vector2 operator/(const double& other) const;
	double x, y;
};

}

#endif // VECTOR2_H
