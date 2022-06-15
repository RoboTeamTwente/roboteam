#include <gtest/gtest.h>
#include <roboteam_utils/Field.hpp>
#include <roboteam_utils/Random.h>

using namespace rtt;

// Function to test if the values correspond to each other
void testCoherence(const Field& f) {
    // Left play area must be inside play area
    ASSERT_DOUBLE_EQ(f.leftPlayArea.getLeft(), f.playArea.getLeft());
    ASSERT_DOUBLE_EQ(f.leftPlayArea.getRight(), f.playArea.getCenter().x);
    ASSERT_DOUBLE_EQ(f.leftPlayArea.getTop(), f.playArea.getTop());
    ASSERT_DOUBLE_EQ(f.leftPlayArea.getBottom(), f.playArea.getBottom());
    // Right play area must be inside play area
    ASSERT_DOUBLE_EQ(f.rightPlayArea.getLeft(), f.playArea.getCenter().x);
    ASSERT_DOUBLE_EQ(f.rightPlayArea.getRight(), f.playArea.getRight());
    ASSERT_DOUBLE_EQ(f.rightPlayArea.getTop(), f.playArea.getTop());
    ASSERT_DOUBLE_EQ(f.rightPlayArea.getBottom(), f.playArea.getBottom());
    // Left play area touches right play area in the middle
    ASSERT_DOUBLE_EQ(f.leftPlayArea.getRight(), f.rightPlayArea.getLeft());

    // Defense areas touch the insides of the play area
    ASSERT_DOUBLE_EQ(f.leftDefenseArea.getLeft(), f.playArea.getLeft());
    ASSERT_DOUBLE_EQ(f.rightDefenseArea.getRight(), f.playArea.getRight());

    // Goal areas touch the outsides of the play area
    ASSERT_DOUBLE_EQ(f.leftGoalArea.getRight(), f.playArea.getLeft());
    ASSERT_DOUBLE_EQ(f.rightGoalArea.getLeft(), f.playArea.getRight());

    // Center of field is indeed 0, 0
    ASSERT_EQ(f.playArea.getCenter(), Vector2(0, 0));

    // Left and right should have same size
    ASSERT_DOUBLE_EQ(f.leftPlayArea.getWidth(), f.rightPlayArea.getWidth());
    ASSERT_DOUBLE_EQ(f.leftPlayArea.getHeight(), f.rightPlayArea.getHeight());
    ASSERT_DOUBLE_EQ(f.leftDefenseArea.getWidth(), f.rightDefenseArea.getWidth());
    ASSERT_DOUBLE_EQ(f.leftDefenseArea.getHeight(), f.rightDefenseArea.getHeight());
    ASSERT_DOUBLE_EQ(f.leftGoalArea.getWidth(), f.rightGoalArea.getWidth());
    ASSERT_DOUBLE_EQ(f.leftGoalArea.getHeight(), f.rightGoalArea.getHeight());
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

    auto f = Field::createField(fieldWidth,
                                fieldHeight,
                                defenseWidth,
                                defenseHeight,
                                goalWidth,
                                goalHeight,
                                boundary,
                                centerRadius,
                                leftPenaltyPoint,
                                rightPenaltyPoint);

    testCoherence(f);

    ASSERT_DOUBLE_EQ(f.playArea.getWidth(), fieldWidth);
    ASSERT_DOUBLE_EQ(f.playArea.getHeight(), fieldHeight);
    ASSERT_DOUBLE_EQ(f.leftDefenseArea.getWidth(), defenseWidth);
    ASSERT_DOUBLE_EQ(f.leftDefenseArea.getHeight(), defenseHeight);
    ASSERT_DOUBLE_EQ(f.leftGoalArea.getWidth(), goalWidth);
    ASSERT_DOUBLE_EQ(f.leftGoalArea.getHeight(), goalHeight);
    ASSERT_DOUBLE_EQ(f.boundaryWidth, boundary);
    ASSERT_DOUBLE_EQ(f.centerCircle.radius, centerRadius);
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
        double centerRadius = SimpleRandom::getDouble(0.1, std::fmin(fieldHeight/2, fieldWidth/2 - defenseWidth));

        double penaltyPointDistanceFromCenter = SimpleRandom::getDouble(centerRadius, fieldWidth/2 - defenseWidth);
        Vector2 leftPenaltyPoint(-penaltyPointDistanceFromCenter, 0);
        Vector2 rightPenaltyPoint(penaltyPointDistanceFromCenter, 0);

        auto f = Field::createField(fieldWidth,
                                    fieldHeight,
                                    defenseWidth,
                                    defenseHeight,
                                    goalWidth,
                                    goalHeight,
                                    boundary,
                                    centerRadius,
                                    leftPenaltyPoint,
                                    rightPenaltyPoint);

        testCoherence(f);

        auto copy = f;

        ASSERT_EQ(f, copy);
    }
}