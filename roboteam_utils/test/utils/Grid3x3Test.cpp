#include <gtest/gtest.h>
#include <roboteam_utils/Grid3x3.hpp>

namespace rtt {

// A 3x3 Grid is actually 3 by 3 cells
TEST(Grid3x3, size) {
    auto grid = Grid3x3<LazyRectangle>(LazyRectangle({1,1},{0,0}));
    ASSERT_EQ(grid.getRowSize(), 3);
    ASSERT_EQ(grid.getColumnSize(), 3);
}

// All easy accessibility functions of 3x3 access correct cell
TEST(Grid3x3, access) {
    auto grid = Grid3x3<FastRectangle>(LazyRectangle({1,1}, {0,0}));

    ASSERT_EQ(grid.topLeftCell(), grid.getCell(0, 0));
    ASSERT_EQ(grid.topMiddleCell(), grid.getCell(1, 0));
    ASSERT_EQ(grid.topRightCell(), grid.getCell(2, 0));
    ASSERT_EQ(grid.middleLeftCell(), grid.getCell(0, 1));
    ASSERT_EQ(grid.middleMiddleCell(), grid.getCell(1, 1));
    ASSERT_EQ(grid.middleRightCell(), grid.getCell(2, 1));
    ASSERT_EQ(grid.bottomLeftCell(), grid.getCell(0, 2));
    ASSERT_EQ(grid.bottomMiddleCell(), grid.getCell(1, 2));
    ASSERT_EQ(grid.bottomRightCell(), grid.getCell(2, 2));
}

} // namespace rtt