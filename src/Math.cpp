#include "roboteam_utils/Vector2.h"
#include "roboteam_utils/Math.h"
#include <cmath>

namespace rtt {

double toDegrees(double radians) {
    return radians * (180/M_PI);
}

double toRadians(double degrees) {
    return degrees * (M_PI/180);
}

double cleanAngle(double angle){
	if (angle <= -M_PI){
		return fmod(angle-M_PI, (2*M_PI))+M_PI;
	} else if(angle > M_PI){
		return fmod(angle+M_PI, (2*M_PI))-M_PI;
	} else {
		return angle;
	}
}

roboteam_utils::Vector2 worldToRobotFrame(roboteam_utils::Vector2 requiredv, double rotation){
    roboteam_utils::Vector2 robotRequiredv;
    robotRequiredv.x=requiredv.x*cos(-rotation)-requiredv.y*sin(-rotation);
    robotRequiredv.y=requiredv.x*sin(-rotation)+requiredv.y*cos(-rotation);
	return robotRequiredv;
}

double computeAngle(roboteam_utils::Vector2 robotPos, roboteam_utils::Vector2 faceTowardsPos) {
	roboteam_utils::Vector2 differenceVector = faceTowardsPos - robotPos;
	return differenceVector.angle();
}

bool isBetweenAngles(double a1, double a2, double testAngle) {
	if (a2 > a1) {
		if (testAngle >= a1 && testAngle <= a2) {
			return true;
		} else {
			return false;
		}
	} else if (a1 > a2) {
		if (testAngle >= a2 && testAngle <= a1) {
			return false;
		} else {
			return true;
		}
	} else {
		return false;
	}
}

double getClockwiseAngle(double a1, double a2) {
	a1 = cleanAngle(a1);
	a2 = cleanAngle(a2);
	double result = a1 - a2;
	if (result < 0) {result += 2*M_PI;}
	return result;
}

double getCounterClockwiseAngle(double a1, double a2) {
	a1 = cleanAngle(a1);
	a2 = cleanAngle(a2);
	double result = a2 - a1;
	if (result < 0) {result += 2*M_PI;}
	return result;
}

} // rtt
