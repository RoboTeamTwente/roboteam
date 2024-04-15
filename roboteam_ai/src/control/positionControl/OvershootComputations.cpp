// The implementation of this is based on Sumatra/modules/common/src/main/java/edu/tigers/sumatra/trajectory/DestinationForTimedPositionCalc.java
#include "control/positionControl/OvershootComputations.h"

#include <cmath>

#include "utilities/Constants.h"
namespace rtt::ai::control {

Vector2 OvershootComputations::overshootingDestination(Vector2& startPosition, Vector2& endPosition, Vector2& startVelocity, double maxVelocity, double maxAcceleration,
                                                       double targetTime) {
    Vector2 distance = endPosition - startPosition;
    double increment = M_PI_4 * 0.5;
    double alpha = M_PI_4;

    TimedPos1D x{0, 0};
    TimedPos1D y{0, 0};

    while (increment > 1e-7) {
        auto cosAlpha = std::cos(alpha);
        auto sinAlpha = std::sin(alpha);
        // If for example the keeper is already at the correct y coordinate, y.time will be almost 0. Decreasing won't make this increase.
        // This will prevent laggy computations. Maybe implement in the normal stuff as well??
        // No clue why we need this, tigers doesnt ? :{}
        // TODO: FIgure out what is happening
        if (cosAlpha <= 0.01 || sinAlpha <= 0.01) {
            break;
        }

        x = getTimedPos1D(distance.x, startVelocity.x, maxVelocity * cosAlpha, maxAcceleration * cosAlpha, targetTime);
        y = getTimedPos1D(distance.y, startVelocity.y, maxVelocity * sinAlpha, maxAcceleration * sinAlpha, targetTime);

        double diff = abs(x.time - y.time);
        if (diff < 1e-3) {
            break;
        }
        alpha += (x.time > y.time) ? -increment : increment;

        increment *= 0.5;
    }
    return Vector2(x.pos + startPosition.x, y.pos + startPosition.y);
}

double OvershootComputations::slowestDirectTime(double distance, double initialVelocity, double maxAcceleration) {
    double deceleration = (initialVelocity >= 0.0) ? -maxAcceleration : maxAcceleration;
    double squareRoot = std::sqrt(initialVelocity * initialVelocity + 2.0 * deceleration * distance);
    return (initialVelocity >= 0.0) ? ((-initialVelocity + squareRoot) / deceleration) : ((-initialVelocity - squareRoot) / deceleration);
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
        return std::make_optional(TimedPos1D(distance + decelerationDistance, accelerationTime + constantVelocityTimeTooSlow + decelerationTime));
    }

    double constantVelocityDistanceEarly = remainingDistance - decelerationDistance;
    double constantVelocityTimeEarly = constantVelocityDistanceEarly / maxVelocity;
    if (constantVelocityTimeEarly >= 0.0 && accelerationTime + constantVelocityTimeEarly + decelerationTime <= targetTime) {
        return std::make_optional(TimedPos1D(distance, accelerationTime + constantVelocityTimeEarly + decelerationTime));
    }

    double remainingTime = targetTime - accelerationTime;
    double decelerationTimeDirect = std::sqrt(2.0 * (remainingDistance - remainingTime * maxVelocity) / deceleration);
    double constantVelocityTimeDirect = remainingTime - decelerationTimeDirect;
    if (constantVelocityTimeDirect > 0.0 && decelerationTimeDirect < decelerationTime) {
        double finalVelocity = maxVelocity + deceleration * decelerationTimeDirect;
        double finalDecelerationTime = -finalVelocity / deceleration;
        return std::make_optional(TimedPos1D(distance + 0.5 * finalVelocity * finalDecelerationTime, targetTime + finalDecelerationTime));
    }
    return std::nullopt;
}

TimedPos1D OvershootComputations::fastestDirectTriangular(double distance, double initialVelocity, double maxVelocity, double maxAcceleration, double deceleration,
                                                          double targetTime) {
    if ((maxVelocity >= 0.0) == (initialVelocity >= maxVelocity)) {
        double time = -initialVelocity / deceleration;
        return TimedPos1D(0.5 * initialVelocity * time, time);
    }
    double acceleration = -deceleration;

    double sqrtTooSlow = std::sqrt(2.0 * acceleration * distance + initialVelocity * initialVelocity);
    double accelerationTimeTooSlow = (maxVelocity >= 0.0) ? ((-initialVelocity + sqrtTooSlow) / acceleration) : ((-initialVelocity - sqrtTooSlow) / acceleration);
    if (accelerationTimeTooSlow >= targetTime) {
        double finalVelocityTooSlow = initialVelocity + acceleration * accelerationTimeTooSlow;
        double decelerationTimeTooSlow = std::abs(finalVelocityTooSlow / acceleration);
        return TimedPos1D(distance + 0.5 * finalVelocityTooSlow * decelerationTimeTooSlow, accelerationTimeTooSlow + decelerationTimeTooSlow);
    }

    double sqEarly = ((distance * acceleration) + (0.5 * initialVelocity * initialVelocity)) / (maxAcceleration * maxAcceleration);
    double decelerationTimeEarly = sqEarly > 0.0 ? std::sqrt(sqEarly) : 0.0;
    double finalVelocityEarly = acceleration * decelerationTimeEarly;
    double accelerationTimeEarly = (finalVelocityEarly - initialVelocity) / acceleration;
    if (accelerationTimeEarly + decelerationTimeEarly <= targetTime) {
        return TimedPos1D(distance, accelerationTimeEarly + decelerationTimeEarly);
    }

    double sqDirect = std::sqrt(2.0 * acceleration * (acceleration * targetTime * targetTime - 2.0 * distance + 2.0 * targetTime * initialVelocity));
    double accelerationTimeDirect = targetTime - sqDirect / (2.0 * maxAcceleration);
    double finalVelocityDirect = initialVelocity + acceleration * accelerationTimeDirect;
    double remainingTimeDirect = finalVelocityDirect / acceleration;
    double accelerationDistanceDirect = 0.5 * (initialVelocity + finalVelocityDirect) * accelerationTimeDirect;
    double remainingDistanceDirect = 0.5 * finalVelocityDirect * remainingTimeDirect;
    return TimedPos1D(accelerationDistanceDirect + remainingDistanceDirect, accelerationTimeDirect + remainingTimeDirect);
}

TimedPos1D OvershootComputations::getTimedPos1D(double distance, double initialVelocity, double maxVelocity, double maxAcceleration, double targetTime) {
    targetTime = std::min(targetTime, 10.0);
    double deceleration = (initialVelocity >= 0.0) ? -maxAcceleration : maxAcceleration;
    double zeroVelocityDistance = 0.5 * initialVelocity * (-initialVelocity / deceleration);
    double maxFinalVelocity = (distance >= 0.0) ? maxVelocity : -maxVelocity;

    if ((distance >= 0.0) != (initialVelocity > 0.0) || (distance >= 0.0) == (zeroVelocityDistance < distance) ||
        slowestDirectTime(distance, initialVelocity, maxAcceleration) >= targetTime) {
        return fastestDirectTrapezoidal(distance, initialVelocity, maxFinalVelocity, maxAcceleration, deceleration, targetTime)
            .value_or(fastestDirectTriangular(distance, initialVelocity, maxFinalVelocity, maxAcceleration, deceleration, targetTime));
    } else {
        double breakingTime = std::abs(initialVelocity / maxAcceleration);
        TimedPos1D timed = fastestDirectTriangular(distance - zeroVelocityDistance, 0.0, -maxFinalVelocity, maxAcceleration, deceleration, targetTime - breakingTime);
        return TimedPos1D(timed.pos + zeroVelocityDistance, timed.time + breakingTime);
    }
}
}  // namespace rtt::ai::control