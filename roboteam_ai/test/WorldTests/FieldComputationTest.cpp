#include <gtest/gtest.h>
#include <helpers/FieldHelper.h>
#include <roboteam_utils/Random.h>
#include <world/FieldComputations.h>

#include <roboteam_utils/Field.hpp>

namespace rtt {
using namespace rtt::world;
using namespace rtt::ai;

double MAXIMUM_DIFFERENCE = 0.000001;

rtt::Field createTestField() { return Field::createField(12, 9, 1.8, 3.6, 0.18, 1.8, 0.3, 0.5, {-2, 0}, {2, 0}); }

/* Checks whether the margins of the functions that return an Polygon are all in outwards direction by checking if the actual size of the area
 * matches with the expected size of the area. */
TEST(FieldComputationTest, outwards_margin) {
    Field testField = createTestField();

    // Check if the area of the field matches when the field is not changed, when it is expanded and when it is shrinked.
    double expectedFieldSize = testField.playArea.width() * testField.playArea.height();
    double actualFieldSize = FieldComputations::getFieldEdge(testField).area();
    EXPECT_TRUE(fabs(expectedFieldSize - actualFieldSize) < MAXIMUM_DIFFERENCE);
    expectedFieldSize = (testField.playArea.width() + 2.0) * (testField.playArea.height() + 2.0);
    actualFieldSize = FieldComputations::getFieldEdge(testField, 1.0).area();
    EXPECT_TRUE(fabs(expectedFieldSize - actualFieldSize) < MAXIMUM_DIFFERENCE);
    expectedFieldSize = (testField.playArea.width() - 2.0) * (testField.playArea.height() - 2.0);
    actualFieldSize = FieldComputations::getFieldEdge(testField, -1.0).area();
    EXPECT_TRUE(fabs(expectedFieldSize - actualFieldSize) < MAXIMUM_DIFFERENCE);

    // Check if the area of our defence area matches when it is not changed, when it is expanded and when it is shrinked.
    double defenceAreaWidth = testField.leftDefenseArea.width();
    double defenceAreaHeight = testField.leftDefenseArea.height();
    double expectedDefenceAreaSize = defenceAreaWidth * defenceAreaHeight;
    double actualDefenceAreaSize = FieldComputations::getDefenseArea(testField, true, 0.0, 0.0).area();
    EXPECT_TRUE(fabs(expectedDefenceAreaSize - actualDefenceAreaSize) < MAXIMUM_DIFFERENCE);
    expectedDefenceAreaSize = (defenceAreaWidth + 1.5) * (defenceAreaHeight + 2.0);
    actualDefenceAreaSize = FieldComputations::getDefenseArea(testField, true, 1.0, 0.5).area();
    EXPECT_TRUE(fabs(expectedDefenceAreaSize - actualDefenceAreaSize) < MAXIMUM_DIFFERENCE);
    expectedDefenceAreaSize = (defenceAreaWidth - 0.75) * (defenceAreaHeight - 0.5);
    actualDefenceAreaSize = FieldComputations::getDefenseArea(testField, true, -0.25, -0.5).area();
    EXPECT_TRUE(fabs(expectedDefenceAreaSize - actualDefenceAreaSize) < MAXIMUM_DIFFERENCE);

    // Check if the area of the opponents defence area matches when it is not changed, when it is expanded and when it is shrinked.
    defenceAreaWidth = testField.rightDefenseArea.width();
    defenceAreaHeight = testField.rightDefenseArea.height();
    expectedDefenceAreaSize = defenceAreaWidth * defenceAreaHeight;
    actualDefenceAreaSize = FieldComputations::getDefenseArea(testField, false, 0.0, 0.0).area();
    EXPECT_TRUE(fabs(expectedDefenceAreaSize - actualDefenceAreaSize) < MAXIMUM_DIFFERENCE);
    expectedDefenceAreaSize = (defenceAreaWidth + 1.5) * (defenceAreaHeight + 2.0);
    actualDefenceAreaSize = FieldComputations::getDefenseArea(testField, false, 1.0, 0.5).area();
    EXPECT_TRUE(fabs(expectedDefenceAreaSize - actualDefenceAreaSize) < MAXIMUM_DIFFERENCE);
    expectedDefenceAreaSize = (defenceAreaWidth - 0.75) * (defenceAreaHeight - 0.5);
    actualDefenceAreaSize = FieldComputations::getDefenseArea(testField, false, -0.25, -0.5).area();
    EXPECT_TRUE(fabs(expectedDefenceAreaSize - actualDefenceAreaSize) < MAXIMUM_DIFFERENCE);

    // Check if the area of our goal area matches when it is not changed, when it is expanded and when it is shrinked.
    double goalAreaWidth = testField.leftGoalArea.width();
    double goalAreaHeight = testField.leftGoalArea.height();
    double expectedGoalAreaSize = goalAreaWidth * goalAreaHeight;
    double actualGoalAreaSize = FieldComputations::getGoalArea(testField, true, 0.0, false).area();
    EXPECT_TRUE(fabs(expectedGoalAreaSize - actualGoalAreaSize) < MAXIMUM_DIFFERENCE);
    expectedGoalAreaSize = (goalAreaWidth + 0.08) * (goalAreaHeight + 0.08);
    actualGoalAreaSize = FieldComputations::getGoalArea(testField, true, 0.04, true).area();
    EXPECT_TRUE(fabs(expectedGoalAreaSize - actualGoalAreaSize) < MAXIMUM_DIFFERENCE);
    expectedGoalAreaSize = (goalAreaWidth - 0.04) * (goalAreaHeight - 0.08);
    actualGoalAreaSize = FieldComputations::getGoalArea(testField, true, -0.04, false).area();
    EXPECT_TRUE(fabs(expectedGoalAreaSize - actualGoalAreaSize) < MAXIMUM_DIFFERENCE);

    // Check if the area of the opponents goal area matches when it is not changed, when it is expanded and when it is shrinked.
    expectedGoalAreaSize = goalAreaWidth * goalAreaHeight;
    actualGoalAreaSize = FieldComputations::getGoalArea(testField, false, 0.0, false).area();
    EXPECT_TRUE(fabs(expectedGoalAreaSize - actualGoalAreaSize) < MAXIMUM_DIFFERENCE);
    expectedGoalAreaSize = (goalAreaWidth + 0.08) * (goalAreaHeight + 0.08);
    actualGoalAreaSize = FieldComputations::getGoalArea(testField, false, 0.04, true).area();
    EXPECT_TRUE(fabs(expectedGoalAreaSize - actualGoalAreaSize) < MAXIMUM_DIFFERENCE);
    expectedGoalAreaSize = (goalAreaWidth - 0.04) * (goalAreaHeight - 0.08);
    actualGoalAreaSize = FieldComputations::getGoalArea(testField, false, -0.04, false).area();
    EXPECT_TRUE(fabs(expectedGoalAreaSize - actualGoalAreaSize) < MAXIMUM_DIFFERENCE);
}

TEST(FieldComputationTest, goal_distance) {
    Field testField = createTestField();

    /* Test cases with a point on the goal, point below the goal, point not on the goal but with y-coordinates between both goal
     * endings and point not on the goal with y-coordinates not between both goal endings. */
    Vector2 testPoint = (testField.leftGoalArea.bottomRight() + testField.leftGoalArea.topRight()) / 2;
    double expectedDistance = FieldComputations::getDistanceToGoal(testField, true, testPoint);
    double actualDistance = 0.0;
    EXPECT_NEAR(expectedDistance, actualDistance, MAXIMUM_DIFFERENCE);
    testPoint = testField.rightGoalArea.bottomLeft() + Vector2(0, -3);
    expectedDistance = FieldComputations::getDistanceToGoal(testField, false, testPoint);
    actualDistance = 3.0;
    EXPECT_NEAR(expectedDistance, actualDistance, MAXIMUM_DIFFERENCE);
    testPoint = Vector2(2, 3.6);
    expectedDistance = FieldComputations::getDistanceToGoal(testField, true, testPoint);
    actualDistance = sqrt(2.7 * 2.7 + 8 * 8);
    EXPECT_NEAR(expectedDistance, actualDistance, MAXIMUM_DIFFERENCE);
    testPoint = Vector2(4, 0.5);
    expectedDistance = FieldComputations::getDistanceToGoal(testField, false, testPoint);
    actualDistance = 2.0;
    EXPECT_NEAR(expectedDistance, actualDistance, MAXIMUM_DIFFERENCE);
}

TEST(FieldComputationTest, total_goal_angle) {
    Field testField = createTestField();

    /* Test cases with a point on the goal, point below the goal, point not on the goal with y-coordinates not between both goal
     * endings, point not on the goal with y-coordinates between both goal endings and point that has an angle of 90 degrees at the
     * goal side. */
    Vector2 testPoint = (testField.rightGoalArea.bottomLeft() + testField.rightGoalArea.topLeft()) / 2;
    double expectedAngle = FieldComputations::getTotalGoalAngle(testField, false, testPoint);
    double actualAngle = M_PI;
    EXPECT_NEAR(expectedAngle, actualAngle, MAXIMUM_DIFFERENCE);

    testPoint = testField.leftGoalArea.bottomRight() + Vector2(0, -3);
    expectedAngle = FieldComputations::getTotalGoalAngle(testField, true, testPoint);
    actualAngle = 0.0;
    EXPECT_NEAR(expectedAngle, actualAngle, MAXIMUM_DIFFERENCE);

    testPoint = testField.playArea.topLeft();
    expectedAngle = FieldComputations::getTotalGoalAngle(testField, false, testPoint);
    actualAngle = atan((4.5 + 0.9) / 12) - atan((4.5 - 0.9) / 12);
    EXPECT_NEAR(expectedAngle, actualAngle, MAXIMUM_DIFFERENCE);

    testPoint = testField.rightGoalArea.topLeft() + Vector2(0, -0.4);
    expectedAngle = FieldComputations::getTotalGoalAngle(testField, true, testPoint);
    actualAngle = atan(0.4 / 12) + atan((1.8 - 0.4) / 12);
    EXPECT_NEAR(expectedAngle, actualAngle, MAXIMUM_DIFFERENCE);

    testPoint = testField.rightGoalArea.bottomLeft() + Vector2(1.2, 0);
    expectedAngle = FieldComputations::getTotalGoalAngle(testField, false, testPoint);
    actualAngle = atan(1.8 / 1.2);
    EXPECT_NEAR(expectedAngle, actualAngle, MAXIMUM_DIFFERENCE);
}

TEST(FieldComputationTest, line_intersection_with_defence_area) {
    Field testField = createTestField();

    /* Test cases where only an intersection happens with the corner, where a LineSegment is closely parallel to the defence area but
     * does not intersect, where a LineSegment has infinitely many intersections with the boundary, where a LineSegment ends closely
     * before the defence area, where a LineSegment only intersect once with boundary of the defence area and 2 test cases where a
     * LineSegment intersects twice with the boundary of the defence area. */
    Vector2 startTestLine = testField.leftDefenseArea.bottomRight() + Vector2(1.5, 0.5);
    Vector2 endTestLine = testField.leftDefenseArea.bottomRight() + Vector2(-0.5, -1.5);
    std::shared_ptr<Vector2> actualIntersection = FieldComputations::lineIntersectionWithDefenseArea(testField, true, startTestLine, endTestLine, 0.5);
    Vector2 expectedIntersection = testField.leftDefenseArea.bottomRight() + Vector2(0.5, -0.5);
    EXPECT_EQ(*actualIntersection, expectedIntersection);
    startTestLine = testField.rightDefenseArea.topLeft() + Vector2(-0.5, -0.499);
    endTestLine = testField.rightDefenseArea.topRight() + Vector2(0.5, -0.499);
    actualIntersection = FieldComputations::lineIntersectionWithDefenseArea(testField, false, startTestLine, endTestLine, -0.5);
    EXPECT_EQ(actualIntersection, nullptr);
    startTestLine = testField.rightDefenseArea.bottomLeft() + Vector2(0.0, 0.5);
    endTestLine = testField.rightDefenseArea.topLeft();
    actualIntersection = FieldComputations::lineIntersectionWithDefenseArea(testField, false, startTestLine, endTestLine, 0.0);
    EXPECT_EQ(*actualIntersection, startTestLine);
    startTestLine = testField.rightGoalArea.leftLine().center();
    endTestLine = (testField.leftDefenseArea.topRight() + testField.leftDefenseArea.bottomRight()) / 2;
    actualIntersection = FieldComputations::lineIntersectionWithDefenseArea(testField, true, startTestLine, endTestLine, -0.1);
    EXPECT_EQ(actualIntersection, nullptr);
    endTestLine = testField.leftDefenseArea.topLeft() * 2 - testField.leftDefenseArea.bottomLeft();
    startTestLine = testField.leftDefenseArea.bottomRight() * 0.99 + endTestLine * 0.01;
    actualIntersection = FieldComputations::lineIntersectionWithDefenseArea(testField, true, startTestLine, endTestLine, 0.0);
    expectedIntersection = (testField.leftDefenseArea.topLeft() + testField.leftDefenseArea.topRight()) / 2;
    EXPECT_EQ(*actualIntersection, expectedIntersection);
    startTestLine = testField.rightDefenseArea.topRight() + Vector2(-0.5, 0.2);
    endTestLine = testField.rightDefenseArea.bottomLeft() + Vector2(0.6, -0.2);
    actualIntersection = FieldComputations::lineIntersectionWithDefenseArea(testField, false, startTestLine, endTestLine, 0.2);
    EXPECT_EQ(*actualIntersection, startTestLine);
    Vector2 intersectStart = testField.leftDefenseArea.bottomLeft() * 0.8 + testField.leftDefenseArea.bottomLeft() * 0.2;
    Vector2 intersectEnd = testField.leftDefenseArea.bottomRight() * 0.25 + testField.leftDefenseArea.topRight() * 0.75;
    startTestLine = intersectStart * 10 - intersectEnd * 9;
    endTestLine = intersectEnd * 10 - intersectStart * 9;
    actualIntersection = FieldComputations::lineIntersectionWithDefenseArea(testField, true, startTestLine, endTestLine, 0.0);
    EXPECT_EQ(*actualIntersection, intersectStart);
}

TEST(FieldComputationTest, projectionTests) {
    Field field = createTestField();
    Vector2 belowGoal = field.leftDefenseArea.bottomLeft();
    Vector2 aboveGoal = field.leftDefenseArea.topLeft();
    Vector2 topPenalty = field.leftDefenseArea.topRight();

    // Random point in our defense area
    auto pointInDefenseArea = Vector2(SimpleRandom::getDouble(aboveGoal.x, topPenalty.x), SimpleRandom::getDouble(belowGoal.y, aboveGoal.y));
    EXPECT_TRUE(field.leftDefenseArea.contains(pointInDefenseArea));

    auto projectedPoint = FieldComputations::projectPointOutOfDefenseArea(field, pointInDefenseArea);
    // Since the projectedPoint can be inside the defense area with ROBOT_RADIUS, we use that as margin
    EXPECT_FALSE(field.leftDefenseArea.contains(projectedPoint, -stp::control_constants::ROBOT_RADIUS));

    auto pointOutsideField = Vector2(field.playArea.left() - 0.05, field.playArea.top() + 0.05);
    EXPECT_FALSE(field.playArea.contains(pointOutsideField));
    projectedPoint = FieldComputations::projectPointInField(field, pointOutsideField);
    EXPECT_TRUE(field.playArea.contains(projectedPoint));

    auto pointBehindGoal = field.leftGoalArea.rightLine().center() + Vector2(-0.10, 0);
    projectedPoint = FieldComputations::projectPointInField(field, pointBehindGoal);
    EXPECT_TRUE(field.leftDefenseArea.contains(projectedPoint));

    auto pointBehindDefArea = Vector2(6.047, -1.12139);
    projectedPoint = FieldComputations::projectPointToValidPosition(field, pointBehindDefArea);
}

TEST(FieldComputationTest, projectionOnLineTests) {
    Field field = createTestField();
    auto line = LineSegment(Vector2(), field.rightGoalArea.leftLine().center());
    auto pointToProject = Vector2(5.5, -1);

    auto projectedPoint = FieldComputations::projectPointToValidPositionOnLine(field, pointToProject, line.start, line.end);
    EXPECT_TRUE(line.isOnLine(projectedPoint));

    auto pointOutsideField = Vector2(1, -4.6);
    line = LineSegment(pointOutsideField, Vector2());
    projectedPoint = FieldComputations::projectPointIntoFieldOnLine(field, pointOutsideField, pointOutsideField, Vector2());
    EXPECT_TRUE(field.playArea.contains(projectedPoint));
    EXPECT_TRUE(line.isOnLine(projectedPoint));
}
}  // namespace rtt