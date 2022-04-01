#pragma once

#include <roboteam_utils/Vector2.h>

namespace rtt {

// This structure contains all the information about any ball in a field
typedef struct Ball {
    Vector2 position;               // (m) Position on the field
    double height = 0.0;            // (m) The vertical position of the ball relative to the field
    Vector2 velocity;               // (m/s) The velocity of the ball
    double verticalVelocity = 0.0;  // (m/s) The vertical velocity
    Vector2 expectedEndPosition;    // (m) Expected position of the ball after it stopped moving
    bool isVisible = false;         // If the ball is currently visible
    unsigned int area;              // The amount of pixels this ball has in the camera
} Ball;

} // namespace rtt