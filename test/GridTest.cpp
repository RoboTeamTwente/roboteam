//
// Created by jesse on 18-05-20.
//

//
// Created by rolf on 22-01-20.
//

#include <gtest/gtest.h>
#include <roboteam_utils/Grid.h>

namespace rtt{

    // Check to see if the grid contains the expected number of data points for the step size
    TEST(Grid, numStepsTest){
        Grid grid = Grid(0, 0, 1, 1, 3, 4);
        int sizeOfGrid = 0;
        for (auto nestedPoints : grid.getPoints()) {
            sizeOfGrid += nestedPoints.size();
        }
        EXPECT_EQ(sizeOfGrid,grid.getNumStepsX() * grid.getNumStepsY());

    }

    // Test to see if any of the points in the grid are not within the allowed region
    TEST(Grid, offSetTest) {
        // The points should be within the box defined by x \in [2, 3] and y \in [2,3]
        Grid grid = Grid(2, 2, 1, 1, 4, 4);
        for (auto nestedPoints : grid.getPoints()) {
            for (auto point : nestedPoints) {
                EXPECT_TRUE(point.x >= 2 && point.x < 3);
                EXPECT_TRUE(point.y >= 2 && point.y < 3);
            }
        }
    }



}
