#pragma once

#include "Vector2.h"

namespace rtt {

namespace clip {

constexpr double X_MIN = -4.5;
constexpr double X_MAX = 4.5;
constexpr double Y_MIN = -3.0;
constexpr double Y_MAX = 3.0;
    
/**
 * @brief Enum used for CS code calculatoin
 * 
 */
enum CSCode {
    INSIDE = 0,
    LEFT = 1,
    RIGHT = 2,
    BOTTOM = 4,
    TOP = 8,
};

/**
 * @brief Returns the CS code for the parameters x and y
 * 
 * Literally: 
 *  int code = CSCode::INSIDE;
 *      if (x < X_MIN) code |= CSCode::LEFT;
 *      if (x > X_MAX) code |= CSCode::RIGHT;
 *      if (y < Y_MIN) code |= CSCode::BOTTOM;
 *      if (y > Y_MAX) code |= CSCode::TOP;
 * 
 * @param x X coordinate
 * @param y Y coordinate
 * @return int The CS Code that matches these parameters
 */
int getCSCode(double x, double y);

/**
 * @brief Clips the two vectors between valid points
 * 
 * @param point0 Point 1
 * @param point1 Point 2
 * @return true If both points are on the field, and nothing got changed
 * @return false If boh points are outside, hence the line is rejected
 */
bool cohenSutherlandClip(Vector2& point0, Vector2& point1);
 
}

}