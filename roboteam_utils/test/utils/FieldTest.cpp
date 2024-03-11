#include <gtest/gtest.h>
#include <roboteam_utils/Random.h>

#include <roboteam_utils/Field.hpp>

using namespace rtt;

// Function to test if the values correspond to each other
void testCoherence(const Field& f) {
    // Left play area must be inside play area
    ASSERT_DOUBLE_EQ(f.leftPlayArea.left(), f.playArea.left());
    ASSERT_DOUBLE_EQ(f.leftPlayArea.right(), f.playArea.center().x);
    ASSERT_DOUBLE_EQ(f.leftPlayArea.top(), f.playArea.top());
    ASSERT_DOUBLE_EQ(f.leftPlayArea.bottom(), f.playArea.bottom());
    // Right play area must be inside play area
    ASSERT_DOUBLE_EQ(f.rightPlayArea.left(), f.playArea.center().x);
    ASSERT_DOUBLE_EQ(f.rightPlayArea.right(), f.playArea.right());
    ASSERT_DOUBLE_EQ(f.rightPlayArea.top(), f.playArea.top());
    ASSERT_DOUBLE_EQ(f.rightPlayArea.bottom(), f.playArea.bottom());
    // Left play area touches right play area in the middle
    ASSERT_DOUBLE_EQ(f.leftPlayArea.right(), f.rightPlayArea.left());

    // Defense areas touch the insides of the play area
    ASSERT_DOUBLE_EQ(f.leftDefenseArea.left(), f.playArea.left());
    ASSERT_DOUBLE_EQ(f.rightDefenseArea.right(), f.playArea.right());

    // Goal areas touch the outsides of the play area
    ASSERT_DOUBLE_EQ(f.leftGoalArea.right(), f.playArea.left());
    ASSERT_DOUBLE_EQ(f.rightGoalArea.left(), f.playArea.right());

    // Center of field is indeed 0, 0
    ASSERT_EQ(f.playArea.center(), Vector2(0, 0));

    // Left and right should have same size
    ASSERT_DOUBLE_EQ(f.leftPlayArea.width(), f.rightPlayArea.width());
    ASSERT_DOUBLE_EQ(f.leftPlayArea.height(), f.rightPlayArea.height());
    ASSERT_DOUBLE_EQ(f.leftDefenseArea.width(), f.rightDefenseArea.width());
    ASSERT_DOUBLE_EQ(f.leftDefenseArea.height(), f.rightDefenseArea.height());
    ASSERT_DOUBLE_EQ(f.leftGoalArea.width(), f.rightGoalArea.width());
    ASSERT_DOUBLE_EQ(f.leftGoalArea.height(), f.rightGoalArea.height());
}

TEST(FieldTest, instantiation) {
    double fieldWidth = 12;
    double fieldHeight = 9;
    double defenseWidth = 1.8;
    double defenseHeight = 3.6;
    double goalWidth = 0.18;
    double goalHeight = 1.8;
    double boundary = 0.3;
    double centerRadius = 0.5;
    Vector2 leftPenaltyPoint(-2, 0);
    Vector2 rightPenaltyPoint(2, 0);

    auto f = Field::createField(fieldWidth, fieldHeight, defenseWidth, defenseHeight, goalWidth, goalHeight, boundary, centerRadius, leftPenaltyPoint, rightPenaltyPoint);

    const double DELTA = 1e-12;
    testCoherence(f);
    ASSERT_NEAR(f.playArea.width(), fieldWidth, DELTA);
    ASSERT_NEAR(f.playArea.height(), fieldHeight, DELTA);
    ASSERT_NEAR(f.leftDefenseArea.width(), defenseWidth, DELTA);
    ASSERT_NEAR(f.leftDefenseArea.height(), defenseHeight, DELTA);
    ASSERT_NEAR(f.leftGoalArea.width(), goalWidth, DELTA);
    ASSERT_NEAR(f.leftGoalArea.height(), goalHeight, DELTA);
    ASSERT_NEAR(f.boundaryWidth, boundary, DELTA);
    ASSERT_NEAR(f.centerCircle.radius, centerRadius, DELTA);
    ASSERT_EQ(f.centerCircle.center, Vector2(0.0, 0.0));
}

TEST(FieldTest, equals) {
    for (int i = 0; i < 50; i++) {
        double fieldWidth = SimpleRandom::getDouble(3, 24);
        double fieldHeight = SimpleRandom::getDouble(2, 15);
        double defenseWidth = SimpleRandom::getDouble(0.5, fieldWidth);
        double defenseHeight = SimpleRandom::getDouble(0.3, fieldHeight);
        double goalWidth = SimpleRandom::getDouble(0.1, 1);
        double goalHeight = SimpleRandom::getDouble(0.1, defenseHeight);
        double boundary = SimpleRandom::getDouble(0.1, 5);
        double centerRadius = SimpleRandom::getDouble(0.1, std::fmin(fieldHeight / 2, fieldWidth / 2 - defenseWidth));

        double penaltyPointDistanceFromCenter = SimpleRandom::getDouble(centerRadius, fieldWidth / 2 - defenseWidth);
        Vector2 leftPenaltyPoint(-penaltyPointDistanceFromCenter, 0);
        Vector2 rightPenaltyPoint(penaltyPointDistanceFromCenter, 0);

        auto f = Field::createField(fieldWidth, fieldHeight, defenseWidth, defenseHeight, goalWidth, goalHeight, boundary, centerRadius, leftPenaltyPoint, rightPenaltyPoint);

        testCoherence(f);

        auto copy = f;

        ASSERT_EQ(f, copy);
    }
}

TEST(FieldTest, edgeCases) {
    // Zero width and height
    Field zeroField = Field::createField(0, 0, 0, 0, 0, 0, 0, 0, Vector2(0, 0), Vector2(0, 0));
    testCoherence(zeroField);
}