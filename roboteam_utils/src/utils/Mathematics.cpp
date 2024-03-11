#include <cmath>

#include "Vector2.h"

namespace rtt {

double toDegrees(double radians) { return radians * (180 / M_PI); }

double toRadians(double degrees) { return degrees * (M_PI / 180); }

double cleanAngle(double angle) {
    if (angle <= -M_PI) {
        return fmod(angle - M_PI, (2 * M_PI)) + M_PI;
    } else if (angle > M_PI) {
        return fmod(angle + M_PI, (2 * M_PI)) - M_PI;
    } else {
        return angle;
    }
}

}  // namespace rtt
