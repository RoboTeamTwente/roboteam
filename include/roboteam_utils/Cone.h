#pragma once

#include "roboteam_utils/Vector2.h"
#include "roboteam_utils/Draw.h"

#include <vector>
#include <string>

namespace rtt {

class Cone {
public:
	Cone(Vector2 startPoint, Vector2 centerPoint, double distance);
	Cone(Vector2 startPoint, Vector2 side1, Vector2 side2);
	bool IsWithinCone(Vector2 point);
	bool IsWithinCone(Vector2 point, double marginRadius);
	bool IsWithinField(Vector2 point);
	Vector2 ClosestPointOnSide(Vector2 point, Vector2 closeTo);
	Vector2 SecondClosestPointOnSide(Vector2 point);
	Vector2 ClosestPointOnSideTwoCones(Cone otherCone, Vector2 point, Vector2 closeTo, Draw drawer, std::vector<std::string> names);
	static Vector2 LineIntersection(Vector2 line1Start, Vector2 line1Dir, Vector2 line2Start, Vector2 line2Dir);
	bool DoConesOverlap(Cone otherCone);
	Cone MergeCones(Cone otherCone);
	~Cone();

	Vector2 start;
	Vector2 center;
	double radius;
	double angle;
	Vector2 side1;
	Vector2 side2;
private:
	// Draw drawer;
};

} // rtt