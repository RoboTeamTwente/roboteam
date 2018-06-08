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

Vector2 limitAngleDiff(Vector2 vector1, Vector2 vector2, double maxAngleDiff) {
    double angleDiff = cleanAngle(vector1.angle() - vector2.angle());
    if (angleDiff > maxAngleDiff) {
        vector1 = vector2.scale(vector1.length() / vector2.length()).rotate(maxAngleDiff);
    } else if (angleDiff < -maxAngleDiff) {
        vector1 = vector2.scale(vector1.length() / vector2.length()).rotate(-maxAngleDiff);
    }
    return vector1;
}

bool isPointInCircle(Vector2 center, double radius, Vector2 point) {
	double xDiffSqr = (point.x-center.x)*(point.x-center.x);
	double yDiffSqr = (point.y-center.y)*(point.y-center.y);
	double radiusSqr = radius*radius;
	return ((xDiffSqr + yDiffSqr) < (radiusSqr));
}

Vector2 worldToRobotFrame(Vector2 requiredv, double rotation){
    Vector2 robotRequiredv;
    robotRequiredv.x=requiredv.x*cos(-rotation)-requiredv.y*sin(-rotation);
    robotRequiredv.y=requiredv.x*sin(-rotation)+requiredv.y*cos(-rotation);
	return robotRequiredv;
}

double computeAngle(Vector2 robotPos, Vector2 faceTowardsPos) {
	Vector2 differenceVector = faceTowardsPos - robotPos;
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

double smoothStep(const double& x) {
	if (x<0) {
		return 0;
	} else if (x>1) {
		return 1;
	} else {
		// x³(6x²-15x+10) -> smooth sigmoid-like function between 0 and 1;
		return x * x * x * (x * (x * 6 - 15) + 10);
	}
}

	/**
	 * https://en.wikipedia.org/wiki/Distance_from_a_point_to_a_line
	 * Calculates the shortest distance from a fixed point to any point on a fixed infinite line in Euclidean geometry.
	 * @param P1 Defines one point on the line
	 * @param P2 Defines another point on the line
	 * @param pos The point of which we want to know the distance to the line
	 * @returns the shortest distance from the point to the line as a double
	 */
	double distanceFromPointToLine(Vector2 P1, Vector2 P2, Vector2 pos){
		return fabs( (P2.y-P1.y)*pos.x - (P2.x-P1.x)*pos.y + P2.x*P1.y - P2.y*P1.x ) / sqrt( pow(P2.y-P1.y, 2) + pow(P2.x-P1.x, 2) );
	}

	/**
	 * https://stackoverflow.com/questions/1811549/perpendicular-on-a-line-from-a-given-point
	 * @param P1 Defines one point on the line
	 * @param P2 Defines another point on the line
	 * @param pos The point which we want to project on the line
	 * @return the projection of pos on the line
	 */
	Vector2 projectPointOntoLine(Vector2 P1, Vector2 P2, Vector2 pos){
		double k = ((P2.y-P1.y) * (pos.x-P1.x) - (P2.x-P1.x) * (pos.y-P1.y)) / (pow(P2.y-P1.y,2) + pow(P2.x-P1.x, 2));
		float x = pos.x - k * (P2.y-P1.y);
		float y = pos.y + k * (P2.x-P1.x);
		return Vector2(x, y);
	}

} // rtt
