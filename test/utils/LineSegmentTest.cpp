#include <gtest/gtest.h>

#include <roboteam_utils/LineSegment.h>
#include <roboteam_utils/Line.h>

using namespace rtt;

TEST(LineSegmentTests, firstIntersects) {
    // The firstIntersects function should return the closest int
    LineSegment a({ -2, 0 }, { 2, 0 });

    Line b({ 0, 2 }, { 0, -2 });
    ASSERT_EQ(a.getClosestPointToLine(b).value(), Vector2(0, 0));
    Line c({ 50, 1 }, { -50, -1 });
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