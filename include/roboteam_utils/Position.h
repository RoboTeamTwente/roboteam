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
	Vector2 location();
	Position translate(const Vector2& vec);
	Position rotate(double rot_vel);
	Position move(const Vector2& vec, double rot_vel);
	double x, y, rot;
};

}

#endif // POSITION_H
