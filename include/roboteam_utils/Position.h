#ifndef POSITION_H
#define POSITION_H

#include "roboteam_utils/Vector2.h"

namespace roboteam_utils
{

class Position
{
public:
	Position(): x(0), y(0), rot(0) {}
	Position(double x, double y, double rot) : x(x), y(y), rot(rot) {}
	~Position();
	Vector2 location() const;
	double getRot() const;
	Position translate(const Vector2& vec) const;
	Position rotate(double rot_vel) const;
	Position move(const Vector2& vec, double rot_vel) const;
    bool real() const;
	bool operator==(const Position& other) const;
	bool operator!=(const Position& other) const;
	Position operator+(const Position& other) const;
	Position operator-(const Position& other) const;
	Position operator*(const double& other) const;
	Position scale(double scalar) const;
	double x, y, rot;
};

}

#endif // POSITION_H
