#include <gtest/gtest.h>
#include <roboteam_utils/Random.h>

#include <roboteam_utils/World.hpp>

using namespace rtt;

World randomWorld() {
    return World{.timePoint = static_cast<unsigned long>(SimpleRandom::getInt(0, 1000)),
                 .id = static_cast<unsigned int>(SimpleRandom::getInt(0, 1000)),
                 .ball = Ball{.position = Vector2(SimpleRandom::getDouble(-5, 5), SimpleRandom::getDouble(-5, 5)),
                              .velocity = Vector2(SimpleRandom::getDouble(-5, 5), SimpleRandom::getDouble(-5, 5)),
                              .expectedEndPosition = Vector2(SimpleRandom::getDouble(-5, 5), SimpleRandom::getDouble(-5, 5))},
                 .yellowRobots = {Robot{.id = SimpleRandom::getInt(0, 15),
                                        .position = Vector2(SimpleRandom::getDouble(-5, 5), SimpleRandom::getDouble(-5, 5)),
                                        .velocity = Vector2(SimpleRandom::getDouble(-5, 5), SimpleRandom::getDouble(-5, 5)),
                                        .angle = SimpleRandom::getDouble(-5, 5),
                                        .capOffset = SimpleRandom::getDouble(-5, 5)}},
                 .blueRobots = {}};
}

Field randomField() {
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

    return Field::createField(fieldWidth, fieldHeight, defenseWidth, defenseHeight, goalWidth, goalHeight, boundary, centerRadius, leftPenaltyPoint, rightPenaltyPoint);
}

TEST(WorldTest, instantiation) {
    World w;
    ASSERT_EQ(w.timePoint, 0);
    ASSERT_EQ(w.id, 0);
    ASSERT_EQ(w.ball, std::nullopt);
}

TEST(WorldTest, equals) {
    for (int i = 0; i < 50; i++) {
        World w = randomWorld();

        auto copy = w;
        ASSERT_EQ(copy, w);
    }
}

TEST(WorldStatesTest, equals) {
    WorldStates ws{.currentWorld = randomWorld(), .field = randomField()};

    auto copy = ws;
    ASSERT_EQ(copy, ws);
}