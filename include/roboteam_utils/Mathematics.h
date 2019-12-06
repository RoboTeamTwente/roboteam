#pragma once

#include "Vector2.h"
#include "type_traits.h"
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

    /**
     * @brief ???
     * 
     * @param vector1 First Vector
     * @param vector2 Second vecor
     * @param maxAngleDiff ?? 
     * @return Vector2 
     */
    Vector2 limitAngleDiff(Vector2 vector1, Vector2 vector2, double maxAngleDiff);

    /**
     * @brief Checks whether a point is in a circle
     * 
     * @param center Center of the circle
     * @param radius Radius of the circle
     * @param point Point to check 
     * @return true True if \ref point is in circle
     * @return false False if \ref point is not in circle
     */
    bool isPointInCircle(Vector2 center, double radius, Vector2 point);

    Vector2 worldToRobotFrame(Vector2 requiredv, double rotation);

    /**
     * @brief Computes the angle between 2 vectors
     * 
     * @param robotPos Vector one
     * @param faceTowardsPos Vector two
     * @return double Computed angle, in radians
     */
    double computeAngle(Vector2 robotPos, Vector2 faceTowardsPos);

    bool isBetweenAngles(double a1, double a2, double testAngle);

    double getClockwiseAngle(double a1, double a2);

    double getCounterClockwiseAngle(double a1, double a2);

    double smoothStep(const double &x);

    double distanceFromPointToLine(Vector2 P1, Vector2 P2, Vector2 pos);

    Vector2 projectPointOntoLine(Vector2 P1, Vector2 P2, Vector2 pos);

    /**
     * @brief Calculates signum of an integral value
     * 
     * @tparam T Type of integral, has to be zero initializable
     * @param x Value ot be signum'd
     * @return constexpr int Signum'd value, constexpr if input is constexpr.
     */
    template<typename T>
    inline constexpr int signum(T x) {
        if constexpr(!type_traits::is_zero_initializable_v<T>) {
            static_assert("Invalid T type in function signum, this value is not zero initializable");
        }

        if constexpr (std::is_signed<T>::value) {
            return (T(0) < x) - (x < T(0)); 
        } else {
            return T(0) < x;
        }
    }

}
