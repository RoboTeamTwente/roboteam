#include "control/positionControl/OvershootComputations.h"

#include <cmath>

#include "utilities/Constants.h"
namespace rtt::ai::control {

Vector2 OvershootComputations::overshootingDestination(Vector2& startPosition, Vector2& endPosition, Vector2& startVelocity, double maxVelocity, double maxAcceleration, double targetTime) {
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

double OvershootComputations::calcSlowestDirectTime(double s, double v0, double aMax) {
    double aDec = (v0 >= 0.0) ? -aMax : aMax;
    double sqrt = std::sqrt(v0 * v0 + 2.0 * aDec * s);
    return (v0 >= 0.0) ? ((-v0 + sqrt) / aDec) : ((-v0 - sqrt) / aDec);
}

std::optional<TimedPos1D> OvershootComputations::calcFastestDirectTrapezoidal(
    double s, double v0, double v1Max, double aMax, double aDec, double tt
) {
    // Full acceleration for s01 to reach v1Max
    double aAcc = (v0 >= v1Max) ? -aMax : aMax;
    double t01 = (v1Max - v0) / aAcc;
    double s01 = 0.5 * (v1Max + v0) * t01;

    if ((s >= 0.0) == (s <= s01)) {
        // We are not able to accel to v1Max before reaching s -> No Trapezoidal form possible
        return std::nullopt;
    }

    double s13 = s - s01;
    double t23 = -v1Max / aDec;
    double s23 = 0.5 * v1Max * t23;

    double t12TooSlow = (s13) / v1Max;
    if (t01 + t12TooSlow >= tt) {
        return std::make_optional(TimedPos1D(s + s23, t01 + t12TooSlow + t23));
    }

    // Determine if "Trapezoidal finishing early"
    double s12Early = s13 - s23;
    double t12Early = s12Early / v1Max;
    if (t12Early >= 0.0 && t01 + t12Early + t23 <= tt) {
        return std::make_optional(TimedPos1D(s, t01 + t12Early + t23));
    }

    // Determine if "Trapezoidal direct hit"
    double t13 = tt - t01;
    double t23Direct = std::sqrt(2.0 * (s13 - t13 * v1Max) / aDec);
    double t12Direct = t13 - t23Direct;
    if (t12Direct > 0.0 && t23Direct < t23) {
        double v3 = v1Max + aDec * t23Direct;
        double t34 = -v3 / aDec;
        return std::make_optional(TimedPos1D(s + 0.5 * v3 * t34, tt + t34));
    }
    return std::nullopt;
}

TimedPos1D OvershootComputations::calcFastestDirectTriangular(
    double s, double v0, double v1Max, double aMax, double aDec, double tt
) {
    // Determining if "Straight too slow"
    if ((v1Max >= 0.0) == (v0 >= v1Max)) {
        double t = -v0 / aDec;
        return TimedPos1D(0.5 * v0 * t, t);
    }
    double aAcc = -aDec;

    // Determining if "Triangular too slow"
    double sqrtTooSlow = std::sqrt(2.0 * aAcc * s + v0 * v0);
    double t01TooSLow = (v1Max >= 0.0) ? ((-v0 + sqrtTooSlow) / aAcc) : ((-v0 - sqrtTooSlow) / aAcc);
    if (t01TooSLow >= tt) {
        double v1TooSlow = v0 + aAcc * t01TooSLow;
        double t12TooSlow = std::abs(v1TooSlow / aAcc);
        return TimedPos1D(s + 0.5 * v1TooSlow * t12TooSlow, t01TooSLow + t12TooSlow);
    }

    // Determining if "Triangular finishing early"
    double sqEarly = ((s * aAcc) + (0.5 * v0 * v0)) / (aMax * aMax);
    double t12Early = sqEarly > 0.0 ? std::sqrt(sqEarly) : 0.0;
    double v1Early = aAcc * t12Early;
    double t01Early = (v1Early - v0) / aAcc;
    if (t01Early + t12Early <= tt) {
        return TimedPos1D(s, t01Early + t12Early);
    }

    // Determining if "Triangular direct hit"
    double sqDirect = std::sqrt(2.0 * aAcc * (aAcc * tt * tt - 2.0 * s + 2.0 * tt * v0));
    double t01Direct = tt - sqDirect / (2.0 * aMax);
    double v1Direct = v0 + aAcc * t01Direct;
    double t13Direct = v1Direct / aAcc;
    double s01Direct = 0.5 * (v0 + v1Direct) * t01Direct;
    double s13Direct = 0.5 * v1Direct * t13Direct;
    return TimedPos1D(s01Direct + s13Direct, t01Direct + t13Direct);
}

TimedPos1D OvershootComputations::getTimedPos1D(double s, double v0, double vMax, double aMax, double tt) {
    tt = std::min(tt, 10.0);
    double aDec = (v0 >= 0.0) ? -aMax : aMax;
    double sZeroVel = 0.5 * v0 * (-v0 / aDec);
    double v1Max = (s >= 0.0) ? vMax : -vMax;

    if ((s >= 0.0) != (v0 > 0.0) || (s >= 0.0) == (sZeroVel < s) || calcSlowestDirectTime(s, v0, aMax) >= tt) {
        // We can directly hit the timed target position
        return calcFastestDirectTrapezoidal(s, v0, v1Max, aMax, aDec, tt).value_or(
            calcFastestDirectTriangular(s, v0, v1Max, aMax, aDec, tt)
        );
    } else {
        // Calculate necessary time to break to zero
        double tBreaking = std::abs(v0 / aMax);
        // Calc the fastest overshoot by starting at sZeroVel in opposed direction with v0=0.0
        TimedPos1D timed = calcFastestDirectTriangular(s - sZeroVel, 0.0, -v1Max, aMax, aDec, tt - tBreaking);
        // Extend TimedPos1D to accommodate breaking
        return TimedPos1D(timed.pos + sZeroVel, timed.time + tBreaking);
    }
}
}  // namespace rtt::ai::control