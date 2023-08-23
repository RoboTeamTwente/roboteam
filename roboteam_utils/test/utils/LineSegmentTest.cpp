#include <gtest/gtest.h>
#include <roboteam_utils/Line.h>
#include <roboteam_utils/LineSegment.h>
#include <roboteam_utils/Random.h>

using namespace rtt;

TEST(LineSegmentTests, firstIntersects) {
    // The firstIntersects function should return the closest int
    LineSegment a({-2, 0}, {2, 0});

    Line b({0, 2}, {0, -2});
    ASSERT_EQ(a.getClosestPointToLine(b).value(), Vector2(0, 0));
    Line c({50, 1}, {-50, -1});
    ASSERT_EQ(a.getClosestPointToLine(c).value(), Vector2(0, 0));

    Line d({2, 2}, {2, -2});
    ASSERT_EQ(a.getClosestPointToLine(d).value(), Vector2(2, 0));
    Line e({4, 2}, {4, -2});
    ASSERT_EQ(a.getClosestPointToLine(e).value(), Vector2(2, 0));

    Line f({-4, -1}, {-1, 1});
    ASSERT_EQ(a.getClosestPointToLine(f).value(), Vector2(-2, 0));

    Line g(a.start, a.end);
    ASSERT_FALSE(a.getClosestPointToLine(g).has_value());
    Line h({-2, 1}, {2, 1});
    ASSERT_FALSE(a.getClosestPointToLine(h).has_value());
}

TEST(LineSegmentTests, center) {
    for (int i = 0; i < 50; i++) {
        auto start = Vector2(SimpleRandom::getDouble(-20, 20), SimpleRandom::getDouble(-20, 20));
        auto end = Vector2(SimpleRandom::getDouble(-20, 20), SimpleRandom::getDouble(-20, 20));

        auto lineSeg = LineSegment(start, end);
        ASSERT_TRUE(lineSeg.isOnLine(lineSeg.center()));

        auto distCenterStart = lineSeg.center().dist(lineSeg.start);
        auto distCenterEnd = lineSeg.center().dist(lineSeg.end);
        ASSERT_TRUE(std::fabs(distCenterStart - distCenterEnd) < 1e-12);  // ASSERT_DOUBLE_EQ can fail here due to small numerical errors for nearly parallel lines,
                                                                          // though these errors should really stay small in practice.
    }
}

TEST(LineSegmentTests, resize) {
    auto null = LineSegment({0, 0}, {0, 0});
    auto resizedNull = null;
    resizedNull.resize(69);
    ASSERT_EQ(null, resizedNull);

    // Resizing to 0 should result in a point
    auto normalLineSeg = LineSegment({1, 0}, {5, 5});
    normalLineSeg.resize(0);
    ASSERT_TRUE(normalLineSeg.isPoint());

    // Create a bunch of random line segments for testing
    for (int i = 0; i < 20; i++) {
        auto start = Vector2(SimpleRandom::getDouble(-20, 20), SimpleRandom::getDouble(-20, 20));
        auto end = Vector2(SimpleRandom::getDouble(-20, 20), SimpleRandom::getDouble(-20, 20));
        // Prevent making a line that is a point
        if (start == end) end = start + 1;
        auto oldLine = LineSegment(start, end);
        double oldLength = oldLine.length();

        // Now create a bunch of random new lengths for testing
        for (int j = 0; j < 20; j++) {
            double resizeValue = SimpleRandom::getDouble(-40, 40);
            double newLength = std::fabs(resizeValue);

            auto newLine = oldLine;
            newLine.resize(resizeValue);

            // The line segment should have the new length after resizing
            EXPECT_NEAR(newLine.length(), newLength, 1e-10);

            // The start and end should not be swapped incorrectly after resizing
            double newStartOldStart = newLine.start.dist(oldLine.start);
            double newStartOldEnd = newLine.start.dist(oldLine.end);
            double newEndOldStart = newLine.end.dist(oldLine.start);
            double newEndOldEnd = newLine.end.dist(oldLine.end);
            if (resizeValue > 0) {
                // Then the new start should be closer to the old start than to the old end, and vice versa
                ASSERT_TRUE(newStartOldStart < newStartOldEnd);
                ASSERT_TRUE(newEndOldEnd < newEndOldStart);
            } else if (resizeValue < 0) {
                // Then the new start should be closer to the old end than to the old start, and vice versa
                ASSERT_TRUE(newStartOldStart > newStartOldEnd);
                ASSERT_TRUE(newEndOldEnd > newEndOldStart);
            }

            if (newLength > oldLength) {
                // Then old points should be on bigger new line
                ASSERT_TRUE(newLine.isOnLine(oldLine.start));
                ASSERT_TRUE(newLine.isOnLine(oldLine.end));
            } else if (newLength < oldLength) {
                // Then new points should be on bigger old line
                ASSERT_TRUE(oldLine.isOnLine(newLine.start));
                ASSERT_TRUE(oldLine.isOnLine(newLine.end));
            }
        }
    }
}