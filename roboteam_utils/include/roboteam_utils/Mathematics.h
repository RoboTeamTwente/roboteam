#pragma once

#include <cmath>

#include "Vector2.h"
#include "type_traits.h"

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
 * @brief Calculates signum of an integral value
 *
 * @tparam T Type of integral, has to be zero initializable
 * @param x Value ot be signum'd
 * @return constexpr int Signum'd value, constexpr if input is constexpr.
 */
template <typename T>
inline constexpr int signum(T x) {
    if constexpr (!type_traits::is_zero_initializable_v<T>) {
        static_assert("Invalid T type in function signum, this value is not zero initializable");
    }

    if constexpr (std::is_signed<T>::value) {
        return (T(0) < x) - (x < T(0));
    } else {
        return T(0) < x;
    }
}

}  // namespace rtt
