#include <gtest/gtest.h>
#include <roboteam_utils/Robot.hpp>
#include <roboteam_utils/Random.h>

using namespace rtt;

TEST(RobotTest, instantiation) {
    Robot r;

    ASSERT_EQ(r.id, 0);
    ASSERT_EQ(r.team, Team::YELLOW);
    ASSERT_EQ(r.angle, Angle());
    ASSERT_DOUBLE_EQ(r.angularVelocity, 0.0);
    ASSERT_FALSE(r.ballSensorSeesBall);
    ASSERT_FALSE(r.ballSensorIsWorking);
    ASSERT_DOUBLE_EQ(r.ballPositionOnSensor, 0.0);
    ASSERT_FALSE(r.dribblerSeesBall);
    ASSERT_DOUBLE_EQ(r.dribblerSpeed, 0.0);
    ASSERT_FALSE(r.xSensIsCalibrated);
    ASSERT_FALSE(r.capacitorIsCharged);
    ASSERT_DOUBLE_EQ(r.signalStrength, 0.0);
    ASSERT_DOUBLE_EQ(r.batteryLevel, 0.0);
    ASSERT_DOUBLE_EQ(r.radius, 0.09);
    ASSERT_DOUBLE_EQ(r.height, 0.15);
    ASSERT_DOUBLE_EQ(r.frontWidth, 0.13);
    ASSERT_DOUBLE_EQ(r.dribblerWidth, 0.1);
    ASSERT_EQ(r.capOffset, Angle());
}

const std::vector<Team> allTeams = { Team::YELLOW, Team::BLUE };

TEST(RobotTest, equals) {
    for (int i = 0; i < 50; i++) {
        Robot r = {
            .id = SimpleRandom::getInt(0, 15),
            .team = *SimpleRandom::getRandomElement(allTeams.begin(), allTeams.end()),
            .position = Vector2(SimpleRandom::getDouble(-20, 20), SimpleRandom::getDouble(-20, 20)),
            .velocity = Vector2(SimpleRandom::getDouble(-5, 5), SimpleRandom::getDouble(-5, 5)),
            .angle = Angle(SimpleRandom::getDouble(0, 7)),
            .angularVelocity = SimpleRandom::getDouble(-3, 3),
            .ballSensorSeesBall = SimpleRandom::getBool(),
            .ballSensorIsWorking = SimpleRandom::getBool(),
            .ballPositionOnSensor = SimpleRandom::getDouble(-0.5, 0.5),
            .dribblerSeesBall = SimpleRandom::getBool(),
            .dribblerSpeed = SimpleRandom::getDouble(0, 50),
            .xSensIsCalibrated = SimpleRandom::getBool(),
            .capacitorIsCharged = SimpleRandom::getBool(),
            .signalStrength = SimpleRandom::getDouble(0, 10),
            .batteryLevel = SimpleRandom::getDouble(18, 24),
            .radius = SimpleRandom::getDouble(0.06, 0.09),
            .height = SimpleRandom::getDouble(0.10, 0.15),
            .frontWidth = SimpleRandom::getDouble(0.10, 0.13),
            .dribblerWidth = SimpleRandom::getDouble(0.7, 0.10),
            .capOffset = Angle(SimpleRandom::getDouble(-M_PI, M_PI))
        };

        auto copy = r;

        ASSERT_EQ(copy, r);
    }
}