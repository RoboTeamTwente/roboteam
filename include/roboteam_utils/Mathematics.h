#pragma once

#include "Vector2.h"
#include <cmath>

namespace rtt {
    
    /**
     * @brief Converts radians to degrees
     * 
     * @param radians Amount of radians
     * @return double Degrees
     */
    double toDegrees(double radians);

    /**
     * @brief Degrees to radians
     * 
     * @param degrees Amount of degrees
     * @return double Radians
     */
    double toRadians(double degrees);

    /**
     * @brief Limits angle between -pi and pi
     * 
     * @param angle Angle to clean
     * @return double Cleaned angle
     */
    double cleanAngle(double angle);

    Vector2 limitAngleDiff(Vector2 vector1, Vector2 vector2, double maxAngleDiff);

    bool isPointInCircle(Vector2 center, double radius, Vector2 point);

    Vector2 worldToRobotFrame(Vector2 requiredv, double rotation);

    double computeAngle(Vector2 robotPos, Vector2 faceTowardsPos);

    bool isBetweenAngles(double a1, double a2, double testAngle);

    double getClockwiseAngle(double a1, double a2);

    double getCounterClockwiseAngle(double a1, double a2);

    double smoothStep(const double &x);

    double distanceFromPointToLine(Vector2 P1, Vector2 P2, Vector2 pos);

    Vector2 projectPointOntoLine(Vector2 P1, Vector2 P2, Vector2 pos);

    template<typename T>
    inline constexpr
    int signum(T x) {
        if constexpr (std::is_signed<T>::value) {
            return (T(0) < x) - (x < T(0)); 
        } else {
            return T(0) < x;
        }
    }

}
