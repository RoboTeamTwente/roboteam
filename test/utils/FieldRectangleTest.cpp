#include <gtest/gtest.h>
#include <roboteam_utils/FieldRectangle.hpp>
#include <roboteam_utils/Random.h>

using namespace rtt;

// Function to test if the values correspond to each other
void testCoherence(const FieldRectangle& fr) {
    // Test if points match
    ASSERT_EQ(fr.bottomLeft(), Vector2(fr.left(), fr.bottom()));
    ASSERT_EQ(fr.topLeft(), Vector2(fr.left(), fr.top()));
    ASSERT_EQ(fr.bottomRight(), Vector2(fr.right(), fr.bottom()));
    ASSERT_EQ(fr.topRight(), Vector2(fr.right(), fr.top()));

    // Test if center is correct
    Vector2 calculatedCenter(
        (fr.right() + fr.left()) / 2,
        (fr.bottom() + fr.top()) / 2
        );
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

TEST(FieldRectangleTest, instantiation) {
    FieldRectangle fr;
    // FR should be a unit rectangle
    ASSERT_EQ(fr.bottomLeft(), Vector2(0.0, 0.0));
    ASSERT_EQ(fr.topRight(), Vector2(1.0, 1.0));
    testCoherence(fr);

    // Creating a rectangle from any opposing corners results in the same one
    FieldRectangle fr1(fr.bottomLeft(), fr.topRight());
    FieldRectangle fr2(fr.topRight(), fr.bottomLeft());
    FieldRectangle fr3(fr.bottomRight(), fr.topLeft());
    FieldRectangle fr4(fr.topLeft(), fr.bottomRight());

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

        ASSERT_TRUE(fr.contains(fr.topLeft()));
        ASSERT_TRUE(fr.contains(fr.topRight()));
        ASSERT_TRUE(fr.contains(fr.bottomLeft()));
        ASSERT_TRUE(fr.contains(fr.bottomRight()));
        ASSERT_TRUE(fr.contains(fr.center()));

        ASSERT_FALSE(fr.contains(Vector2(fr.left() - 1.0, fr.center().y)));
        ASSERT_FALSE(fr.contains(Vector2(fr.right() + 1.0, fr.center().y)));
        ASSERT_FALSE(fr.contains(Vector2(fr.center().x, fr.top() + 1.0)));
        ASSERT_FALSE(fr.contains(Vector2(fr.center().x, fr.bottom() - 1.0)));
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