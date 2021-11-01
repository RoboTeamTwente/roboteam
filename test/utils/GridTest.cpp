//
// Created by jesse on 18-05-20.
//

//
// Created by rolf on 22-01-20.
//

#include <gtest/gtest.h>
#include <roboteam_utils/Grid.h>

namespace rtt {

// Check to see if the grid contains the expected number of data points for the step size
TEST(Grid, numStepsTest) {
    Grid grid = Grid(0, 0, 1, 1, 3, 4);
    int sizeOfGrid = 0;
    for (auto nestedPoints : grid.getPoints()) {
        sizeOfGrid += nestedPoints.size();
    }
    EXPECT_EQ(sizeOfGrid, (grid.getNumSegmentsX() + 1) * (grid.getNumSegmentsY() + 1));
}

// Test to see if any of the points in the grid are not within the allowed region
TEST(Grid, offSetTest) {
    // The points should be within the box defined by x \in [2, 3] and y \in [2,3]
    Grid grid = Grid(2, 2, 1, 1, 4, 4);
    for (auto nestedPoints : grid.getPoints()) {
        for (auto point : nestedPoints) {
            EXPECT_TRUE(point.x >= 2 && point.x <= 3);
            EXPECT_TRUE(point.y >= 2 && point.y <= 3);
        }
    }
}
TEST(Grid, getters) {
    Grid grid = Grid(1, 2, 3, 4, 5, 6);
    EXPECT_EQ(grid.getOffSetX(), 1);
    EXPECT_EQ(grid.getOffSetY(), 2);
    EXPECT_EQ(grid.getRegionWidth(), 3);
    EXPECT_EQ(grid.getRegionHeight(), 4);
    EXPECT_EQ(grid.getNumSegmentsX(), 5);
    EXPECT_EQ(grid.getNumSegmentsY(), 6);
    EXPECT_DOUBLE_EQ(grid.getStepSizeX(), 3 / 5.0);  // TOOD: possible bugs? Is the grid defined to be inclusive edge or not?
    EXPECT_DOUBLE_EQ(grid.getStepSizeY(), 4 / 6.0);
}

}  // namespace rtt
