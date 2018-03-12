#pragma once

#include "roboteam_utils/Vector2.h"
#include <cmath>

namespace rtt {

double toDegrees(double radians);

double toRadians(double degrees);

double cleanAngle(double angle);
Vector2 limitAngleDiff(Vector2 vector1, Vector2 vector2, double maxAngleDiff);
bool isPointInCircle(Vector2 center, double radius, Vector2 point);
Vector2 worldToRobotFrame(Vector2 requiredv, double rotation);
double computeAngle(Vector2 robotPos, Vector2 faceTowardsPos);
bool isBetweenAngles(double a1, double a2, double testAngle);
double getClockwiseAngle(double a1, double a2);
double getCounterClockwiseAngle(double a1, double a2);
double smoothStep(double x);

template <typename T> inline constexpr
int signum(T x, std::false_type is_signed) {
    return T(0) < x;
}
template <typename T> inline constexpr
int signum(T x, std::true_type is_signed) {
    return (T(0) < x) - (x < T(0));
}
template <typename T> inline constexpr
int signum(T x) {
    return signum(x, std::is_signed<T>());
}

}
