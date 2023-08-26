#pragma once

#include <roboteam_utils/Circle.h>
#include <roboteam_utils/Grid.h>
#include <roboteam_utils/Vector2.h>

#include <roboteam_utils/FastRectangle.hpp>
#include <vector>

namespace rtt {
/* This type represents a field, containing a set of rectangles,
 * points and circles, all expressed in meters.
 * The center of the field is (0, 0), and the x-axis goes through the goals */
typedef struct Field {
    Vector2 leftPenaltyPoint;   // Place of left penalty point
    Vector2 rightPenaltyPoint;  // Place of right penalty point

    Circle centerCircle;  // Contains the center point and describes the circle around it

    double boundaryWidth = 0.0;  // Width of the area around the play area

    FastRectangle playArea;          // The whole field in which robots can drive (excluding goal areas)
    FastRectangle leftPlayArea;      // Left side of the field
    FastRectangle rightPlayArea;     // Right side of the field
    FastRectangle leftDefenseArea;   // Left defense area inside left play area
    FastRectangle rightDefenseArea;  // Right defense area inside right play area
    FastRectangle leftGoalArea;      // Left goal area outside the play area
    FastRectangle rightGoalArea;     // Right goal area outside the play area

    // TODO: Put these grids into a single grid of grids
    Grid topLeftGrid;
    Grid topMidGrid;
    Grid topRightGrid;
    Grid middleLeftGrid;
    Grid middleMidGrid;
    Grid middleRightGrid;
    Grid bottomLeftGrid;
    Grid bottomMidGrid;
    Grid bottomRightGrid;

    bool operator==(const Field& other) const;

    /* Function to ensure all variables of the created field are properly set.
     * Assumes no negative sizes are given */
    static Field createField(double fieldWidth, double fieldHeight, double defenseWidth, double defenseHeight, double goalWidth, double goalHeight, double boundaryWidth,
                             double centerCircleRadius, const Vector2& leftPenaltyPoint, const Vector2& rightPenaltyPoint);
} Field;

}  // namespace rtt