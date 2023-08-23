#include <gtest/gtest.h>
#include <roboteam_utils/Random.h>

#include <roboteam_utils/FastRectangle.hpp>

using namespace rtt;

// Function to test if the values correspond to each other
void testCoherence(const FastRectangle& fr) {
    // Test if points match
    ASSERT_EQ(fr.bottomLeft(), Vector2(fr.left(), fr.bottom()));
    ASSERT_EQ(fr.topLeft(), Vector2(fr.left(), fr.top()));
    ASSERT_EQ(fr.bottomRight(), Vector2(fr.right(), fr.bottom()));
    ASSERT_EQ(fr.topRight(), Vector2(fr.right(), fr.top()));

    // Test if center is correct
    Vector2 calculatedCenter(
        (fr.right() + fr.left()) / 2,
        (fr.bottom() + fr.top()) / 2);
    ASSERT_EQ(fr.center(), calculatedCenter);

    // Test if line segments are correct
    ASSERT_EQ(fr.topLine().start, fr.topLeft());
    ASSERT_EQ(fr.topLine().end, fr.topRight());

    ASSERT_EQ(fr.rightLine().start, fr.topRight());
    ASSERT_EQ(fr.rightLine().end, fr.bottomRight());

    ASSERT_EQ(fr.bottomLine().start, fr.bottomRight());
    ASSERT_EQ(fr.bottomLine().end, fr.bottomLeft());

    ASSERT_EQ(fr.leftLine().start, fr.bottomLeft());
    ASSERT_EQ(fr.leftLine().end, fr.topLeft());
}

TEST(FastRectangleTest, instantiation) {
    FastRectangle fr;
    // FR should be a unit rectangle
    ASSERT_EQ(fr.bottomLeft(), Vector2(0.0, 0.0));
    ASSERT_EQ(fr.topRight(), Vector2(1.0, 1.0));
    testCoherence(fr);

    // Creating a rectangle from any opposing corners results in the same one
    FastRectangle fr1(fr.bottomLeft(), fr.topRight());
    FastRectangle fr2(fr.topRight(), fr.bottomLeft());
    FastRectangle fr3(fr.bottomRight(), fr.topLeft());
    FastRectangle fr4(fr.topLeft(), fr.bottomRight());

    ASSERT_EQ(fr, fr1);
    ASSERT_EQ(fr, fr2);
    ASSERT_EQ(fr, fr3);
    ASSERT_EQ(fr, fr4);
}

TEST(FastRectangleTest, contains) {
    FastRectangle fr(1, 1, -1, -1);

    // 'contains' tests for points within the rectangle, not on the edge
    // (Makes sense, as using <= instead of < makes no sense with doubles (doubles often fail ==))
    ASSERT_FALSE(fr.contains(fr.topLeft()));
    ASSERT_FALSE(fr.contains(fr.topRight()));
    ASSERT_FALSE(fr.contains(fr.bottomLeft()));
    ASSERT_FALSE(fr.contains(fr.bottomRight()));

    const double almostOne = 0.99999999;
    ASSERT_TRUE(fr.contains(fr.center()));
    ASSERT_TRUE(fr.contains({ almostOne, almostOne }));
    ASSERT_TRUE(fr.contains({ almostOne, -almostOne }));
    ASSERT_TRUE(fr.contains({ -almostOne, almostOne }));
    ASSERT_TRUE(fr.contains({ -almostOne, -almostOne }));
}

TEST(FastRectangleTest, containsMargin) {
    FastRectangle fr(1, 1, -1, -1);

    const double DELTA = 0.00000001;

    // Where 'contains' says points on the edge are not contained, with a margin they are
    ASSERT_TRUE(fr.contains(fr.topLeft(), DELTA));
    ASSERT_TRUE(fr.contains(fr.topRight(), DELTA));
    ASSERT_TRUE(fr.contains(fr.bottomLeft(), DELTA));
    ASSERT_TRUE(fr.contains(fr.bottomRight(), DELTA));
    ASSERT_TRUE(fr.contains(fr.center(), DELTA));
}

TEST(FastRectangleTest, project) {
    FastRectangle rect(1, 1, -1, -1);

    // Projecting point inside shape does not return different point
    ASSERT_EQ(rect.center(), rect.project(rect.center()));
    ASSERT_EQ(rect.topLeft(), rect.project(rect.topLeft()));
    ASSERT_EQ(rect.topRight(), rect.project(rect.topRight()));
    ASSERT_EQ(rect.bottomLeft(), rect.project(rect.bottomLeft()));
    ASSERT_EQ(rect.bottomRight(), rect.project(rect.bottomRight()));

    // Projection of point is always in/on rectangle
    const double DELTA = 0.0000001;

    ASSERT_TRUE(rect.contains(rect.project(Vector2(2, 2)), DELTA));
    ASSERT_TRUE(rect.contains(rect.project(Vector2(2, -2)), DELTA));
    ASSERT_TRUE(rect.contains(rect.project(Vector2(-2, 2)), DELTA));
    ASSERT_TRUE(rect.contains(rect.project(Vector2(-2, -2)), DELTA));
}

TEST(FastRectangleTest, equals) {
    for (int i = 0; i < 50; i++) {
        Vector2 a(SimpleRandom::getDouble(-20, 20), SimpleRandom::getDouble(-20, 20));
        Vector2 b(SimpleRandom::getDouble(-20, 20), SimpleRandom::getDouble(-20, 20));
        FastRectangle r(a, b);

        auto copy = r;

        ASSERT_EQ(r, copy);
    }
}