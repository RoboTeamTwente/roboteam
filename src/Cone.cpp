#include "../include/roboteam_utils/Cone.h"
#include "../include/roboteam_utils/Vector2.h"
#include "GeometryFieldSize.pb.h"
#include <cmath>
#include "../include/roboteam_utils/Mathematics.h"

namespace rtt {

Cone::Cone(Vector2 startPoint, Vector2 centerPoint, double distance) {
	start = startPoint;
	center = centerPoint;
	radius = distance;
	if (radius > 0) {
		angle = 2 * atan((radius) / (center-start).length());
	} else {
		// ROS_WARN("radius for cone invalid!");
	}
	side1 = (center-start).rotate(0.5*angle);
	side2 = (center-start).rotate(-0.5*angle);
}

Cone::Cone(Vector2 startPoint, Vector2 sideA, Vector2 sideB) {
	start = startPoint;
	if (sideA.length()>sideB.length()) {
		side1 = sideA;
		side2 = sideB.stretchToLength(sideA.length());
	} else {
		side1 = sideA.stretchToLength(sideB.length());
		side2 = sideB;
	}
	angle = cleanAngle(side1.angle() - side2.angle());
	center = side2.rotate(0.5*angle) + start;
	angle = fabs(angle);
	radius = (start + side1 - center).length();
}

bool Cone::IsWithinCone(Vector2 point) const noexcept {
	return IsWithinCone(point,0.0);
}

bool Cone::IsWithinCone(Vector2 point, double marginRadius) const noexcept {
	Vector2 vectorToPoint = point-start;
	Vector2 vectorToCenter = center-start;
	double extraAngle = atan(marginRadius / vectorToPoint.length());

	if (fabs(rtt::cleanAngle(vectorToPoint.angle()-vectorToCenter.angle())) < (0.5*angle + extraAngle)) {
		return true;
	} else {
		return false;
	}
}


bool Cone::IsWithinField(Vector2 point) const noexcept {
	/**
	 * Guys, no return value is undefined behavior and can crash your program
	 * Please pay attention to this stuff
	 */
	return false;
	// std::cerr << "Cone::IsWithinField is DEPRECATED" << std::endl;
	// roboteam_msgs::GeometryFieldSize field = LastWorld::get_field();
	// double fieldLimitX = field.field_length / 2.0;
	// double fieldLimitY = field.field_width / 2.0;
	// if (point.x > fieldLimitX || point.x < -fieldLimitX || point.y > fieldLimitY || point.y < -fieldLimitY) {
	// 	return false;
	// } else {
	// 	return true;
	// }
}

Vector2 Cone::ClosestPointOnSide(Vector2 point, Vector2 closeTo) const noexcept {
	if (!IsWithinCone(point)) {
		// ROS_WARN("This point is not inside the cone");
		return point;
	}
	Vector2 option1 = side1.scale(10 / side1.length()).closestPointOnVector(start, point);
	Vector2 option2 = side2.scale(10 / side2.length()).closestPointOnVector(start, point);

	double option1Dist = (closeTo-option1).length();
	double option2Dist = (closeTo-option2).length();

	if (option1Dist <= option2Dist) {
		if (IsWithinField(option1)) return option1;
		else if (IsWithinField(option2)) return option2;
		else return option1;
	} else {
		if (IsWithinField(option2)) return option2;
		else if (IsWithinField(option1)) return option1;
		else return option2;
	}
}

Vector2 Cone::SecondClosestPointOnSide(Vector2 point) const noexcept {
	if (!IsWithinCone(point)) {
		std::cout << "This point is not inside the cone" << std::endl;
		return point;
	}
	Vector2 vectorToPoint = point-start;
	Vector2 vectorToCenter = center-start;
	double pointAngle = cleanAngle(vectorToPoint.angle()-vectorToCenter.angle());
	Vector2 option1 = vectorToCenter.rotate(angle).scale(vectorToPoint.length() / vectorToCenter.length()) + start;
	Vector2 option2 = vectorToCenter.rotate(-angle).scale(vectorToPoint.length() / vectorToCenter.length()) + start;

	if (pointAngle >= 0) {
		if (IsWithinField(option2)) return option2;
		else if (IsWithinField(option1)) return option1;
	} else {
		if (IsWithinField(option1)) return option1;
		else if (IsWithinField(option2)) return option2;
	}
	return point;
}

Vector2 Cone::ClosestPointOnSideTwoCones(Cone otherCone, Vector2 point, Vector2 closeTo) const noexcept {
	if (!(this->IsWithinCone(point) && otherCone.IsWithinCone(point))) {
		std::cout << "This point is not inside either of the cones" << std::endl;

		return point;
	}

	std::vector<Vector2> intersections{
	  LineIntersection(start, side1, otherCone.start, otherCone.side1),
	  LineIntersection(start, side1, otherCone.start, otherCone.side2),
	  LineIntersection(start, side2, otherCone.start, otherCone.side1),
	  LineIntersection(start, side2, otherCone.start, otherCone.side2), 
	};

	auto calc = [&](size_t idx) {
		return (intersections[idx] - closeTo).length();
	};

	std::vector<double> costs { 
		calc(0), calc(1), calc(2), calc(3)
	};


	return intersections.at(
		distance(costs.begin(), min_element(costs.begin(),costs.end()))
	);
}

Vector2 Cone::LineIntersection(Vector2 line1Start, Vector2 line1Dir, Vector2 line2Start, Vector2 line2Dir) noexcept {
	float slope1 = line1Dir.y / line1Dir.x;
	float slope2 = line2Dir.y / line2Dir.x;
	float intersectX = (slope1*line1Start.x - slope2*line2Start.x - line1Start.y + line2Start.y) / (slope1 - slope2);
	float intersectY = slope1 * (intersectX - line1Start.x) + line1Start.y;
	return Vector2(intersectX, intersectY);
}

Vector2 Cone::LineIntersection(Vector2 lineStart, Vector2 lineEnd, bool sideA) const noexcept {
	if (sideA) {
		return LineIntersection(start, side1, lineStart, lineEnd - lineStart);
	} else {
		return LineIntersection(start, side2, lineStart, lineEnd - lineStart);
	}
	
}

bool Cone::DoConesOverlap(Cone otherCone) const noexcept {
	return this->IsWithinCone(otherCone.side1 + otherCone.start) || this->IsWithinCone(otherCone.side2 + otherCone.start) || otherCone.IsWithinCone(side1 + start) || otherCone.IsWithinCone(side2 + start);
}

Cone Cone::MergeCones(Cone otherCone) const noexcept {
	if (!DoConesOverlap(otherCone)) {
		std::cout << "no overlap" << std::endl;
		return *this;
	}

	if (this->IsWithinCone(otherCone.side1 + otherCone.start) && this->IsWithinCone(otherCone.side2 + otherCone.start)) {
		return *this;
	}
	if (otherCone.IsWithinCone(side1 + start) && otherCone.IsWithinCone(side2 + start)) {
		return otherCone;

	}

	double angleDiff1 = fabs(cleanAngle(side1.angle() - otherCone.side2.angle()));
	double angleDiff2 = fabs(cleanAngle(side2.angle() - otherCone.side1.angle()));

	if (angleDiff1 > angleDiff2) {
		return Cone(start, side1, otherCone.side2);
	} else {
		return Cone(start, otherCone.side1, side2);
	}
}

} // rtt
