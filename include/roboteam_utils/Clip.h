#pragma once

#include "Vector2.h"

namespace rtt {

namespace clip {

constexpr double X_MIN = -4.5;
constexpr double X_MAX = 4.5;
constexpr double Y_MIN = -3.0;
constexpr double Y_MAX = 3.0;
    
typedef enum CSCode {
    INSIDE = 0,
    LEFT = 1,
    RIGHT = 2,
    BOTTOM = 4,
    TOP = 8
} CSCode;

int getCSCode(double x, double y);

bool cohenSutherlandClip(roboteam_utils::Vector2& point0, roboteam_utils::Vector2& point1);
 
}

}