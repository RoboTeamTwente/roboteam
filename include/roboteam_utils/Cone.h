#pragma once

#include "roboteam_utils/Vector2.h"
#include "roboteam_utils/Draw.h"

namespace rtt {

class Cone {
public:
	Cone(roboteam_utils::Vector2 startPoint, roboteam_utils::Vector2 centerPoint, double distance);
	Cone(roboteam_utils::Vector2 startPoint, roboteam_utils::Vector2 side1, roboteam_utils::Vector2 side2);
	bool IsWithinCone(roboteam_utils::Vector2 point);
	bool IsWithinCone(roboteam_utils::Vector2 point, double marginRadius);
	bool IsWithinField(roboteam_utils::Vector2 point);
	roboteam_utils::Vector2 ClosestPointOnSide(roboteam_utils::Vector2 point, roboteam_utils::Vector2 closeTo);
	roboteam_utils::Vector2 SecondClosestPointOnSide(roboteam_utils::Vector2 point);
	roboteam_utils::Vector2 ClosestPointOnSideTwoCones(Cone otherCone, roboteam_utils::Vector2 point, roboteam_utils::Vector2 closeTo);
	static roboteam_utils::Vector2 LineIntersection(roboteam_utils::Vector2 line1Start, roboteam_utils::Vector2 line1Dir, roboteam_utils::Vector2 line2Start, roboteam_utils::Vector2 line2Dir);
	bool DoConesOverlap(Cone otherCone);
	Cone MergeCones(Cone otherCone);
	~Cone();

	roboteam_utils::Vector2 start;
	roboteam_utils::Vector2 center;
	double radius;
	double angle;
	roboteam_utils::Vector2 side1;
	roboteam_utils::Vector2 side2;
private:
	Draw drawer;
};

} // rtt