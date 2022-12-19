#include <gtest/gtest.h>
#include <roboteam_utils/FastGrid.hpp>
#include <roboteam_utils/Random.h>

namespace rtt {

    // The size should match the given template arguments
    TEST(FastGrid, sizeTest) {
        constexpr unsigned int ROW_SIZE = 3;
        constexpr unsigned int COLUMN_SIZE = 4;
        constexpr unsigned int EXPECTED_SIZE = ROW_SIZE * COLUMN_SIZE;

        auto grid = FastGrid<ROW_SIZE, COLUMN_SIZE, FastRectangle>(LazyRectangle({ 0, 0 }, { 1, 1 }));

        EXPECT_EQ(grid.getPoints().size(), EXPECTED_SIZE);
        EXPECT_EQ(grid.getCells().size(), EXPECTED_SIZE);
        EXPECT_EQ(grid.getRowSize(), ROW_SIZE);
        EXPECT_EQ(grid.getColumnSize(), COLUMN_SIZE);
    }

    // All points from a grid should be within that grid
    TEST(FastGrid, pointsTest) {
        for (int i = 0; i < 20; i++) {
            double left = SimpleRandom::getDouble(-10, 5);
            double right = SimpleRandom::getDouble(left, 10);
            double bottom = SimpleRandom::getDouble(-10, 5);
            double top = SimpleRandom::getDouble(bottom, 10);
            auto gridBounding = LazyRectangle({ left, top }, { right, bottom });

            auto grid = FastGrid<3, 3, FastRectangle>(gridBounding);

            for (const auto& point : grid.getPoints()) {
                ASSERT_TRUE(grid.contains(point));
            }
        }
    }

    // Incorrect construction of grid throws exception
    TEST(FastGrid, constructionException) {
        auto gridBoundary = LazyRectangle({ 1, 1 }, { 0, 0 });

        EXPECT_NO_THROW((FastGrid<1, 1, LazyRectangle>(gridBoundary)));
        EXPECT_THROW((FastGrid<0, 1, LazyRectangle>(gridBoundary)), InvalidGridSizeException);
        EXPECT_THROW((FastGrid<1, 0, LazyRectangle>(gridBoundary)), InvalidGridSizeException);
        EXPECT_THROW((FastGrid<0, 0, LazyRectangle>(gridBoundary)), InvalidGridSizeException);
    }

    // Access to nonexistent cells throws exception
    TEST(FastGrid, noCellException) {
        constexpr int WIDTH = 3;
        constexpr int HEIGHT = 3;
        auto gridBoundary = LazyRectangle({ 1, 1 }, { 0, 0 });
        auto grid = FastGrid<WIDTH, HEIGHT, LazyRectangle>(gridBoundary);

        // Valid cells will not throw
        for (int x = 0; x < WIDTH; x++) {
            for (int y = 0; y < HEIGHT; y++) {
                EXPECT_NO_THROW(grid.getCell(x, y));
            }
        }
        // Invalid cells will throw
        EXPECT_THROW(grid.getCell(WIDTH, 0), InvalidCellLocation);
        EXPECT_THROW(grid.getCell(0, HEIGHT), InvalidCellLocation);
        EXPECT_THROW(grid.getCell(WIDTH, HEIGHT), InvalidCellLocation);
    }

    // All points of a grid should be equally spaced in the region (average should be in the center of the grid)
    TEST(FastGrid, spreadTest) {
        auto gridBoundary = LazyRectangle({ 1, 1 }, { 0, 0 });
        auto grid = FastGrid<3, 4, LazyRectangle>(gridBoundary);

        auto points = grid.getPoints();

        Vector2 sum = std::reduce(points.begin(), points.end(), Vector2(0.0, 0.0));
        Vector2 average = sum / static_cast<double>(points.size());

        grid.getPoints().size();
        grid.getPoints().size();

        EXPECT_EQ(average, grid.center());
    }

} // namespace rtt