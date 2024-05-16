#include <gtest/gtest.h>
#include <roboteam_utils/HalfLine.h>

using namespace rtt;

TEST(HalfLineTest, intersect) {
    Vector2 start(0, 0);
    Vector2 goesThrough(1, 1);
    HalfLine halfLine(start, goesThrough);

    // Line that intersects the half line
    Line line(Vector2(-1, 1), Vector2(1, -1));
    auto intersection = halfLine.intersect(line);
    ASSERT_TRUE(intersection.has_value());
    EXPECT_EQ(intersection.value(), Vector2(0, 0));

    // Line that does not intersect the half line
    Line line2(Vector2(-1, 0), Vector2(0, -2));
    intersection = halfLine.intersect(line2);
    EXPECT_FALSE(intersection.has_value());

    // Line that is parallel to the half line
    Line line3(Vector2(0, 1), Vector2(1, 2));
    intersection = halfLine.intersect(line3);
    EXPECT_FALSE(intersection.has_value());

    // Line that is on the half line
    Line line4(Vector2(-2, -2), Vector2(-1, -1));
    intersection = halfLine.intersect(line4);
    ASSERT_TRUE(intersection.has_value());
}

TEST(HalfLineTest, project) {
    Vector2 start(0, 0);
    Vector2 goesThrough(1, 1);
    HalfLine halfLine(start, goesThrough);

    Vector2 point(1, 0);
    Vector2 projection = halfLine.project(point);
    EXPECT_EQ(projection, Vector2(0.5, 0.5));
}

TEST(HalfLineTest, toLine) {
    Vector2 start(0, 0);
    Vector2 goesThrough(1, 1);
    HalfLine halfLine(start, goesThrough);

    Line line = halfLine.toLine();
    EXPECT_EQ(line.v1, start);
    EXPECT_EQ(line.v2, goesThrough);
}