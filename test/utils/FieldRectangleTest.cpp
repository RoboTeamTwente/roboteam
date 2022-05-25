#include <gtest/gtest.h>
#include <roboteam_utils/FieldRectangle.hpp>
#include <roboteam_utils/Random.h>

using namespace rtt;

// Function to test if the values correspond to each other
void testCoherence(const FieldRectangle& fr) {
    // Test if points match
    ASSERT_EQ(fr.getBottomLeft(), Vector2(fr.getLeft(), fr.getBottom()));
    ASSERT_EQ(fr.getTopLeft(), Vector2(fr.getLeft(), fr.getTop()));
    ASSERT_EQ(fr.getBottomRight(), Vector2(fr.getRight(), fr.getBottom()));
    ASSERT_EQ(fr.getTopRight(), Vector2(fr.getRight(), fr.getTop()));

    // Test if center is correct
    Vector2 calculatedCenter(
        (fr.getRight() + fr.getLeft()) / 2,
        (fr.getBottom() + fr.getTop()) / 2
        );
    ASSERT_EQ(fr.getCenter(), calculatedCenter);

    // Test if line segments are correct
    ASSERT_EQ(fr.getTopSide().start, fr.getTopLeft());
    ASSERT_EQ(fr.getTopSide().end, fr.getTopRight());

    ASSERT_EQ(fr.getRightSide().start, fr.getTopRight());
    ASSERT_EQ(fr.getRightSide().end, fr.getBottomRight());

    ASSERT_EQ(fr.getBottomSide().start, fr.getBottomRight());
    ASSERT_EQ(fr.getBottomSide().end, fr.getBottomLeft());

    ASSERT_EQ(fr.getLeftSide().start, fr.getBottomLeft());
    ASSERT_EQ(fr.getLeftSide().end, fr.getTopLeft());
}

TEST(FieldRectangleTest, instantiation) {
    FieldRectangle fr;
    // FR should be a unit rectangle
    ASSERT_EQ(fr.getBottomLeft(), Vector2(0.0, 0.0));
    ASSERT_EQ(fr.getTopRight(), Vector2(1.0, 1.0));
    testCoherence(fr);

    // Creating a rectangle from any opposing corners results in the same one
    FieldRectangle fr1(fr.getBottomLeft(), fr.getTopRight());
    FieldRectangle fr2(fr.getTopRight(), fr.getBottomLeft());
    FieldRectangle fr3(fr.getBottomRight(), fr.getTopLeft());
    FieldRectangle fr4(fr.getTopLeft(), fr.getBottomRight());

    ASSERT_EQ(fr, fr1);
    ASSERT_EQ(fr, fr2);
    ASSERT_EQ(fr, fr3);
    ASSERT_EQ(fr, fr4);
}

TEST(FieldRectangleTest, contains) {
    for (int i = 0; i < 50; i++) {
        Vector2 a(SimpleRandom::getDouble(-20, 20), SimpleRandom::getDouble(-20, 20));
        Vector2 b(SimpleRandom::getDouble(-20, 20), SimpleRandom::getDouble(-20, 20));

        FieldRectangle fr(a, b);
        testCoherence(fr);

        ASSERT_TRUE(fr.contains(fr.getTopLeft()));
        ASSERT_TRUE(fr.contains(fr.getTopRight()));
        ASSERT_TRUE(fr.contains(fr.getBottomLeft()));
        ASSERT_TRUE(fr.contains(fr.getBottomRight()));
        ASSERT_TRUE(fr.contains(fr.getCenter()));

        ASSERT_FALSE(fr.contains(Vector2(fr.getLeft() - 1.0, fr.getCenter().y)));
        ASSERT_FALSE(fr.contains(Vector2(fr.getRight() + 1.0, fr.getCenter().y)));
        ASSERT_FALSE(fr.contains(Vector2(fr.getCenter().x, fr.getTop() + 1.0)));
        ASSERT_FALSE(fr.contains(Vector2(fr.getCenter().x, fr.getBottom() - 1.0)));
    }
}

TEST(FieldRectangleTest, project) {
    for (int i = 0; i < 50; i++) {
        Vector2 a(SimpleRandom::getDouble(-20, 20), SimpleRandom::getDouble(-20, 20));
        Vector2 b(SimpleRandom::getDouble(-20, 20), SimpleRandom::getDouble(-20, 20));

        FieldRectangle fr(a, b);
        testCoherence(fr);

        for (int j = 0; j < 50; j++) {
            Vector2 p(SimpleRandom::getDouble(-30, 30), SimpleRandom::getDouble(-30, 30));

            ASSERT_TRUE(fr.contains(fr.project(p)));
        }
    }
}

TEST(FieldRectangleTest, equals) {
    for (int i = 0; i < 50; i++) {
        Vector2 a(SimpleRandom::getDouble(-20, 20), SimpleRandom::getDouble(-20, 20));
        Vector2 b(SimpleRandom::getDouble(-20, 20), SimpleRandom::getDouble(-20, 20));
        FieldRectangle r(a, b);

        auto copy = r;

        ASSERT_EQ(r, copy);
    }
}