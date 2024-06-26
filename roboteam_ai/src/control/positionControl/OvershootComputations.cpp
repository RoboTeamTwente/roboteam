// The implementation of this is based on Sumatra/modules/common/src/main/java/edu/tigers/sumatra/trajectory/DestinationForTimedPositionCalc.java
#include "control/positionControl/OvershootComputations.h"

#include <cmath>

#include "utilities/Constants.h"

namespace rtt::ai::control {

std::pair<Vector2, double> OvershootComputations::overshootingDestination(const Vector2& startPosition, const Vector2& endPosition, const Vector2& startVelocity,
                                                                          const double maxVelocity, const double maxAcceleration, const double targetTime) {
    Vector2 distance = endPosition - startPosition;
    double increment = M_PI_4 * 0.5;
    double alpha = M_PI_4;

    TimedPos1D x{0, 0, 0};
    TimedPos1D y{0, 0, 0};

    while (increment > 1e-7) {
        auto cosAlpha = std::cos(alpha);
        auto sinAlpha = std::sin(alpha);
        x = getTimedPos1D(distance.x, startVelocity.x, maxVelocity * cosAlpha, maxAcceleration * cosAlpha, targetTime);
        y = getTimedPos1D(distance.y, startVelocity.y, maxVelocity * sinAlpha, maxAcceleration * sinAlpha, targetTime);
        double diff = abs(x.time - y.time);
        if (diff < 1e-3) {
            break;
        }
        alpha += (x.time > y.time) ? -increment : increment;

        increment *= 0.5;
    }
    return {Vector2(x.pos + startPosition.x, y.pos + startPosition.y), std::max(x.timeToTarget, y.timeToTarget)};
}

double OvershootComputations::slowestDirectTime(double distance, double initialVelocity, double maxAcceleration) {
    double deceleration = (initialVelocity >= 0.0) ? -maxAcceleration : maxAcceleration;
    double squareRoot = std::sqrt(initialVelocity * initialVelocity + 2.0 * deceleration * distance);
    return (initialVelocity >= 0.0) ? ((-initialVelocity + squareRoot) / deceleration) : ((-initialVelocity - squareRoot) / deceleration);
}

TimedPos1D OvershootComputations::fastestDirect(double distance, double initialVelocity, double maxVelocity, double maxAcceleration, double targetTime) {
    double deceleration = (maxVelocity >= 0.0) ? -maxAcceleration : maxAcceleration;
    return fastestDirectTrapezoidal(distance, initialVelocity, maxVelocity, maxAcceleration, deceleration, targetTime)
        .value_or(fastestDirectTriangular(distance, initialVelocity, maxVelocity, maxAcceleration, deceleration, targetTime));
}

std::optional<TimedPos1D> OvershootComputations::fastestDirectTrapezoidal(double distance, double initialVelocity, double maxVelocity, double maxAcceleration, double deceleration,
                                                                          double targetTime) {
    double acceleration = (initialVelocity >= maxVelocity) ? -maxAcceleration : maxAcceleration;
    double accelerationTime = (maxVelocity - initialVelocity) / acceleration;
    double accelerationDistance = 0.5 * (maxVelocity + initialVelocity) * accelerationTime;

    if ((distance >= 0.0) == (distance <= accelerationDistance)) {
        return std::nullopt;
    }

    double remainingDistance = distance - accelerationDistance;
    double decelerationTime = -maxVelocity / deceleration;
    double decelerationDistance = 0.5 * maxVelocity * decelerationTime;

    double constantVelocityTimeTooSlow = remainingDistance / maxVelocity;
    if (accelerationTime + constantVelocityTimeTooSlow >= targetTime) {
        // Target reached after the target time, 'Trapezoidal too slow'
        return std::make_optional(
            TimedPos1D(distance + decelerationDistance, accelerationTime + constantVelocityTimeTooSlow + decelerationTime, accelerationTime + constantVelocityTimeTooSlow));
    }

    double constantVelocityDistanceEarly = remainingDistance - decelerationDistance;
    double constantVelocityTimeEarly = constantVelocityDistanceEarly / maxVelocity;
    if (constantVelocityTimeEarly >= 0.0 && accelerationTime + constantVelocityTimeEarly + decelerationTime <= targetTime) {
        // Target reached before the target time, 'Trapezoidal finishing early'
        // Robot is already at a complete stop before the target time
        return std::make_optional(
            TimedPos1D(distance, accelerationTime + constantVelocityTimeEarly + decelerationTime, accelerationTime + constantVelocityTimeEarly + decelerationTime));
    }

    double remainingTime = targetTime - accelerationTime;
    double decelerationTimeDirect = std::sqrt(2.0 * (remainingDistance - remainingTime * maxVelocity) / deceleration);
    double constantVelocityTimeDirect = remainingTime - decelerationTimeDirect;
    if (constantVelocityTimeDirect >= 0.0 && decelerationTimeDirect <= decelerationTime) {
        // Target reached at the target time, 'Trapezoidal direct hit'
        double finalVelocity = maxVelocity + deceleration * decelerationTimeDirect;
        double finalDecelerationTime = -finalVelocity / deceleration;
        return std::make_optional(TimedPos1D(distance + 0.5 * finalVelocity * finalDecelerationTime, targetTime + finalDecelerationTime, targetTime));
    }
    return std::nullopt;
}

TimedPos1D OvershootComputations::fastestDirectTriangular(double distance, double initialVelocity, double maxVelocity, double maxAcceleration, double deceleration,
                                                          double targetTime) {
    // The "Straight Too Slow" case from the TDP is not needed, as it's already covered in getTimedPos1D, at the start of the function. It's almost the same as the "Straight Too
    // Slow" from the TDP, except that it's a bit more flexible and allows easier return of timeToTarget, which we need for our keeper extension.
    double acceleration = -deceleration;

    double sqrtTooSlow = std::sqrt(2.0 * acceleration * distance + initialVelocity * initialVelocity);
    double accelerationTimeTooSlow = (maxVelocity >= 0.0) ? ((-initialVelocity + sqrtTooSlow) / acceleration) : ((-initialVelocity - sqrtTooSlow) / acceleration);
    if (accelerationTimeTooSlow >= targetTime) {
        // "Triangular too slow", robot can not reach the target in time, does full acceleration till target position
        double finalVelocityTooSlow = initialVelocity + acceleration * accelerationTimeTooSlow;
        double decelerationTimeTooSlow = std::abs(finalVelocityTooSlow / acceleration);
        return TimedPos1D(distance + 0.5 * finalVelocityTooSlow * decelerationTimeTooSlow, accelerationTimeTooSlow + decelerationTimeTooSlow, accelerationTimeTooSlow);
    }

    double sqEarly = (distance * acceleration + 0.5 * initialVelocity * initialVelocity) / (maxAcceleration * maxAcceleration);
    double decelerationTimeEarly = sqEarly > 0.0 ? std::sqrt(sqEarly) : 0.0;
    double finalVelocityEarly = acceleration * decelerationTimeEarly;
    double accelerationTimeEarly = (finalVelocityEarly - initialVelocity) / acceleration;
    if (accelerationTimeEarly + decelerationTimeEarly <= targetTime) {
        // "Triangular finishing early", robot reaches target before target time, comes to a full stop before target time
        return TimedPos1D(distance, accelerationTimeEarly + decelerationTimeEarly, accelerationTimeEarly + decelerationTimeEarly);
    }

    double sqDirect = std::sqrt(2.0 * acceleration * (acceleration * targetTime * targetTime - 2.0 * distance + 2.0 * targetTime * initialVelocity));
    double accelerationTimeDirect = targetTime - sqDirect / (2.0 * maxAcceleration);
    double finalVelocityDirect = initialVelocity + acceleration * accelerationTimeDirect;
    double remainingTimeDirect = finalVelocityDirect / acceleration;
    double accelerationDistanceDirect = 0.5 * (initialVelocity + finalVelocityDirect) * accelerationTimeDirect;
    double remainingDistanceDirect = 0.5 * finalVelocityDirect * remainingTimeDirect;
    // "Triangular direct hit", robot reaches target at target time while still moving
    return TimedPos1D(accelerationDistanceDirect + remainingDistanceDirect, accelerationTimeDirect + remainingTimeDirect, targetTime);
}

TimedPos1D OvershootComputations::getTimedPos1D(double distance, double initialVelocity, double maxVelocity, double maxAcceleration, double targetTime) {
    if (abs(initialVelocity) > maxVelocity) {
        double breakingTimeToMaxVel = ((abs(initialVelocity) - maxVelocity) / maxAcceleration);
        double breakingDistanceToMaxVel = 0.5 * (abs(initialVelocity) + maxVelocity) * breakingTimeToMaxVel;
        if (initialVelocity < 0.0) {
            breakingDistanceToMaxVel *= -1.0;
        }
        auto newTimedPos1D = getTimedPos1D(distance - breakingDistanceToMaxVel, maxVelocity, maxVelocity, maxAcceleration, targetTime - breakingTimeToMaxVel);
        return TimedPos1D(newTimedPos1D.pos + breakingDistanceToMaxVel, newTimedPos1D.time + breakingTimeToMaxVel, newTimedPos1D.timeToTarget + breakingTimeToMaxVel);
    }

    targetTime = std::min(targetTime, 10.0);
    double deceleration = (initialVelocity >= 0.0) ? -maxAcceleration : maxAcceleration;
    double zeroVelocityDistance = 0.5 * initialVelocity * (-initialVelocity / deceleration);
    double maxFinalVelocity = (distance >= 0.0) ? maxVelocity : -maxVelocity;

    if ((distance >= 0.0) != (initialVelocity > 0.0) || (distance >= 0.0) == (zeroVelocityDistance < distance) ||
        slowestDirectTime(distance, initialVelocity, maxAcceleration) >= targetTime) {
        return fastestDirect(distance, initialVelocity, maxFinalVelocity, maxAcceleration, targetTime);
    } else {
        double breakingTime = std::abs(initialVelocity / maxAcceleration);
        TimedPos1D timed = fastestDirect(distance - zeroVelocityDistance, 0.0, -maxFinalVelocity, maxAcceleration, targetTime - breakingTime);
        return TimedPos1D(timed.pos + zeroVelocityDistance, timed.time + breakingTime, timed.timeToTarget + breakingTime);
    }
}
}  // namespace rtt::ai::control