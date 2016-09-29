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
	double dot(const Vector2& other);
	double dist(const Vector2& other);
	double dist2(const Vector2& other);
	Vector2 scale(double scalar);
	Vector2 normalize();
	double length();
	bool operator==(const Vector2& other);
	bool operator!=(const Vector2& other);
	Vector2 operator+(const Vector2& other);
	Vector2 operator-(const Vector2& other);
	Vector2 operator*(const Vector2& other);
	double x, y;
};

}

#endif // VECTOR2_H
