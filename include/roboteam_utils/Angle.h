//
// Created by thijs on 28-2-19.
//

#ifndef ANGLE_H
#define ANGLE_H

#include "Mathematics.h"

namespace rtt {

    class Vector2;

/**
 * The Angle class is a modular class on the interval [-PI, PI) that deals with addition, subtractions, distances and comparisons on this scale.
 * @author Created by: Thijs Luttikhuis <br>
 *         Recreated by: Haico Dorenbos
 * @since 2019-02-28
 */
class Angle {
    public:
        /**
         * Construct the zero Angle, i.e. the Angle with 0 as angle value.
         */
        Angle() = default;

        /**
         * Construct a new Angle instance by using a double value representing the angle value, which will be directly normalized. However you should be cautious with float values
         * close to -MAX_FLOAT and close to MAX_FLOAT.
         * @param angle The given double value. It is allowed to use a value outside the range [-PI, PI).
         */
        Angle(const double &angle);

        /**
         * Construct a new Angle instance by using the angle of a Vector2 (compared to the origin). In case the Angle is the origin then construct the zero Angle.
         * @param vec The given Vector2.
         */
        Angle(const rtt::Vector2 &vec);

        /**
         * Get the angle value in the range [-PI, PI).
         * @return The angle value.
         */
        operator double() const noexcept;

        /**
         * Check what is the shortest direction to move from this angle to the other angle. In case the distance between both angles is 0 then we prefer the positive direction,
         * so in this case we return true. In case the distance between both angles is PI then we prefer the negative direction, so in this case we return false.
         * @param other The other angle
         * @return True if the positive direction is the shortest (which is counterclockwise), false if the negative direction is the shortest (which is clockwise).
         */
        [[nodiscard]] bool rotateDirection(const Angle &other) const noexcept;

        /**
         * Compute the shortest absolute angle difference (modular difference) between this Angle and the other Angle.
         * @param other The other Angle.
         * @return A double value in the range [0, PI].
         */
        [[nodiscard]] double shortestAngleDiff(Angle const &other) const noexcept;

        /**
         * Create a new Vector2 using an absolute distance from the origin and this as given Angle.
         * @param length The distance from the origin, which should be >= 0.
         * @return Vector2 representing this position.
         */
        [[nodiscard]] rtt::Vector2 toVector2(double length = 1.0) const noexcept;

        /**
         * Check if two Angle instances represents the same angle value. This function is protected against double/float rounding issues.
         * @param other The other Angle.
         * @return True if the Angle instances represents the same angle value, false otherwise.
         */
        bool operator==(const Angle &other) const noexcept;

        /**
         * Check if two Angle instances represents the same angle value. This function is protected against double/float rounding issues.
         * @param other The other Angle.
         * @return False if the Angle instances represents the same angle value, true otherwise.
         */
        bool operator!=(const Angle &other) const noexcept;

        /**
         * Compute the addition between this Angle and the other Angle, and directly normalize the angle value.
         * @param other The other Angle.
         * @return A new Angle instance which is the result of the addition.
         */
        Angle operator+(const Angle &other) const noexcept;

        /**
         * Compute the subtraction between this Angle and the other Angle, and directly normalize the angle value.
         * @param other The other Angle.
         * @return A new Angle instance which is the result of the subtraction.
         */
        Angle operator-(const Angle &other) const noexcept;

        /**
         * Add the other Angle to this Angle, and directly normalize the angle value.
         * @param other The other Angle.
         * @return A reference to this updated Angle instance.
         */
        Angle &operator+=(const Angle &other) noexcept;

        /**
         * Subtract the other Angle from this Angle, and directly normalize the angle value.
         * @param other The other Angle.
         * @return A reference to this updated Angle instance.
         */
        Angle &operator-=(const Angle &other) noexcept;

        /**
         * Set the angle value of this Angle instance, which will be directly normalized.
         * @param scalar A double value which represent the new angle value. It is allowed to use a value outside the range [-PI, PI). However you should be cautious with float
         * values close to -MAX_FLOAT and close to MAX_FLOAT.
         * @return A reference to this updated Angle instance.
         */
        Angle &operator=(const double &scalar) noexcept;

    private:
        double angle; // The current angle value, which is a value in the range [-PI, PI).

        /**
         * Normalize the angle value to the range [-PI, PI).
         * @return A reference to this normalized Angle instance.
         */
        Angle normalize() noexcept;
    };

} // rtt

#endif //ANGLE_H
