#pragma once

#include <roboteam_utils/FieldRectangle.hpp>
#include <roboteam_utils/Vector2.h>
#include <roboteam_utils/Grid.h>
#include <roboteam_utils/Circle.h>

#include <vector>

namespace rtt {
/* This type represents a field, containing a set of rectangles,
 * points and circles, all expressed in meters.
 * The center of the field is (0, 0), and the x-axis goes through the goals */
typedef struct Field {

    Vector2 leftPenaltyPoint;   // Place of left penalty point
    Vector2 rightPenaltyPoint;  // Place of right penalty point

    Circle centerCircle;        // Contains the center point and describes the circle around it

    double boundaryWidth = 0.0; // Width of the area around the play area

    FieldRectangle playArea;        // The whole field in which robots can drive (excluding goal areas)
    FieldRectangle leftPlayArea;    // Left side of the field
    FieldRectangle rightPlayArea;   // Right side of the field
    FieldRectangle leftDefenseArea; // Left defense area inside left play area
    FieldRectangle rightDefenseArea;// Right defense area inside right play area
    FieldRectangle leftGoalArea;    // Left goal area outside the play area
    FieldRectangle rightGoalArea;   // Right goal area outside the play area

    bool operator== (const Field& other) const;

    /* Function to ensure all variables of the created field are properly set.
     * Assumes no negative sizes are given */
    static Field createField(double fieldWidth,
                             double fieldHeight,
                             double defenseWidth,
                             double defenseHeight,
                             double goalWidth,
                             double goalHeight,
                             double boundaryWidth,
                             double centerCircleRadius,
                             const Vector2& leftPenaltyPoint,
                             const Vector2& rightPenaltyPoint);
} Field;

} // namespace rtt