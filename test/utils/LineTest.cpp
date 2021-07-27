//
// Recreated by Haico Dorenbos on 12-06-2020.
//

#include <gtest/gtest.h>
#include <math.h>
#include "roboteam_utils/HalfLine.h"
#include "roboteam_utils/Line.h"
#include "roboteam_utils/LineSegment.h"
#include "roboteam_utils/Vector2.h"

using namespace rtt;

const double EPSILON = 1e-2;  // Small difference that is still significant enough to really change a Vector2 position.

/* In this test class we test different methods in the Line classes: Line, LineSegment, HalfLine. Note that testing intersection between Line classes is tested in the
 * LineIntersectionTest class and that Line projections are tested in the LineProjectionTest class. */

TEST(LineTests, relativePosition) {
    /* Test the relativePosition method by using a vertical line (does not change in x direction), horizontal line (does not change in y direction), arbitrary other line (does
    change in both x and y direction). */
    Vector2 start(0.0, 0.0), end(0.0, -EPSILON), linePoint(0.0, 1e6);
    float expected = -1e6 / EPSILON;
    float actual = Line::relativePosition(start, end, linePoint);
    EXPECT_EQ(expected, actual);
    start = {0.0, 0.0}, end = {-1e9, 0.0}, linePoint = {0.0, 0.0};
    expected = 0.0;
    actual = Line::relativePosition(start, end, linePoint);
    EXPECT_EQ(expected, actual);
    start = {-2.0, 8.0}, end = {3.0, -12.0}, linePoint = {0.0, 0.0};
    expected = 0.4;
    actual = Line::relativePosition(start, end, linePoint);
    EXPECT_EQ(expected, actual);
}