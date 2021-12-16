//
// Created by rolf on 19-4-19.
//

#include <gtest/gtest.h>

#include "roboteam_utils/LineSegment.h"
#include "roboteam_utils/Shadow.h"
#include "roboteam_utils/Vector2.h"

using namespace rtt;
TEST(LineTests, Shadow) {
    Vector2 source(1.0, 2.0);  // The light source that is used in every test case.
    Vector2 projectLine_start(-4.0, -6.0), projectLine_end(-4.0, 6.0);
    LineSegment projectLine(projectLine_start, projectLine_end);  // The LineSegments on which the shadow is projected. This LineSegment is also used in every test case.

    // The LineSegments which are the used obstacles in the different test cases.
    Vector2 blockLine1_start(0.0, 2.0), blockLine1_end(0.0, 2.0), blockLine2_start(-1.0, 0.0), blockLine2_end(-1.0, 2.0), blockLine3_start(1.0, 1.0), blockLine3_end(0.0, 0.0),
        blockLine4_start(1.0, 3.0), blockLine4_end(0.0, 2.0), blockLine5_start(0.0, -4.0), blockLine5_end(0.0, 4.0), blockLine6_start(1.0, 2.0), blockLine6_end(1.0, 2.0),
        blockLine7_start(1.0, 0.0), blockLine7_end(1000000000.0, 0.0), blockLine8_start(-1.0, 1.0), blockLine8_end(-3.0, -1.0);
    LineSegment blockLine1(blockLine1_start, blockLine1_end), blockLine2(blockLine2_start, blockLine2_end), blockLine3(blockLine3_start, blockLine3_end),
        blockLine4(blockLine4_start, blockLine4_end), blockLine5(blockLine5_start, blockLine5_end), blockLine6(blockLine6_start, blockLine6_end),
        blockLine7(blockLine7_start, blockLine7_end), blockLine8(blockLine8_start, blockLine8_end);

    // The LineSegments which are the expected shadows in the different test cases, to which the actual shadow is checked against.
    Vector2 checkLine2_start(-4.0, -3.0), checkLine2_end(-4.0, 2.0), checkLine4_start(-4.0, 2.0), checkLine4_end(-4.0, 6.0), checkLine8_start(-4.0, -0.5),
        checkLine8_end(-4.0, -1.75);
    LineSegment checkLine2(checkLine2_start, checkLine2_end), checkLine4(checkLine4_start, checkLine4_end), checkLine8(checkLine8_start, checkLine8_end);

    std::optional<LineSegment> result = Shadow::shadow(source, blockLine1, projectLine);
    EXPECT_FALSE(result.has_value());  // A single point cannot cause a shadow (except if it lies at the source)
    result = Shadow::shadow(source, blockLine2, projectLine);
    EXPECT_TRUE(result.value() == checkLine2);  // Part of the line is covered by a shadow
    result = Shadow::shadow(source, blockLine3, projectLine);
    EXPECT_FALSE(result.has_value());  // Line that does not cause a shadow at all
    result = Shadow::shadow(source, blockLine4, projectLine);
    EXPECT_TRUE(result.value() == checkLine4);  // Only shadow at upper part, projected line on the end of the blockline goes parallel to the projection line
    result = Shadow::shadow(source, blockLine5, projectLine);
    EXPECT_TRUE(result.value() == projectLine);  // Entire project line is shadowed by obstacle
    result = Shadow::shadow(source, blockLine6, projectLine);
    EXPECT_TRUE(result.value() == projectLine);  // The source is in the obstacle so everything is in the shadow
    result = Shadow::shadow(source, blockLine7, projectLine);
    EXPECT_FALSE(result.has_value());  // Line that is very long, but does not cause a shadow at all because it is not in between the project line and source
    result = Shadow::shadow(source, blockLine8, projectLine);
    EXPECT_TRUE(result.value() == checkLine8);
}