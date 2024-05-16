#include <gtest/gtest.h>
#include <roboteam_utils/Angle.h>

#include "TestFixtures/TestFixture.h"
#include "roboteam_utils/Random.h"
#include "world/FieldComputations.h"

namespace cr = rtt::ai::control;
using Vector2 = rtt::Vector2;
using Constants = rtt::ai::Constants;
using ControlUtils = rtt::ai::control::ControlUtils;

TEST(ControlUtils, velocityLimiter) {
    for (int i = 0; i < 200; i++) {
        EXPECT_LE(cr::ControlUtils::velocityLimiter(Vector2(-102 + i * 10, 512 + i * 8)).length(), rtt::ai::Constants::MAX_VEL() + 0.01);
    }

    // check the min velocity as well
    // limit values between 56 and 3000
    for (int i = 0; i < 200; i++) {
        auto limitedVel = cr::ControlUtils::velocityLimiter(Vector2(-102 + i * 10, 512 + i * 8), 3000, 56, false).length();
        EXPECT_GE(limitedVel, 56 - 0.01);
        EXPECT_LE(limitedVel, 3000 + 0.01);
    }

    for (int i = 0; i < 200; i++) {
        auto limitedVel = cr::ControlUtils::velocityLimiter(Vector2(-102 + i * 10, 512 + i * 8), 3000, 56, true).length();
        EXPECT_GE(limitedVel, 0 - 0.01);
        EXPECT_LE(limitedVel, 8 + 0.01);
    }
}

// the function should return weight/distance^2 * normalized vector IF within minDistance, otherwise 0.
TEST(ControlUtils, it_calculates_forces) {
    Vector2 force;

    // distance should be ok. 2/2^2
    force = cr::ControlUtils::calculateForce(Vector2(0, -2), 2, 3);
    EXPECT_DOUBLE_EQ(force.x, 0);
    EXPECT_DOUBLE_EQ(force.y, -0.5);

    // distance should be ok. 2/2^2
    force = cr::ControlUtils::calculateForce(Vector2(0, 2), 2, 3);
    EXPECT_DOUBLE_EQ(force.x, 0);
    EXPECT_DOUBLE_EQ(force.y, 0.5);

    // distance not ok.
    force = cr::ControlUtils::calculateForce(Vector2(0, 3.1), 2, 3);
    EXPECT_DOUBLE_EQ(force.x, 0);
    EXPECT_DOUBLE_EQ(force.y, 0);

    // distance ok. this is a negative force because of negative weight. -2/1^2
    force = cr::ControlUtils::calculateForce(Vector2(0, 1), -2, 3);
    EXPECT_DOUBLE_EQ(force.x, 0);
    EXPECT_DOUBLE_EQ(force.y, -2);
}

TEST(ControlUtils, object_velocity_aimed_at_point) {
    // With a velocity of 0,0 it should always return false
    EXPECT_FALSE(cr::ControlUtils::objectVelocityAimedToPoint({0, 0}, {0, 0}, {1, 0}, 0.3));
    EXPECT_FALSE(cr::ControlUtils::objectVelocityAimedToPoint({0, 0}, {0, 0}, {-1, 0}, 0.3));

    // velocity in wrong direction
    EXPECT_FALSE(cr::ControlUtils::objectVelocityAimedToPoint({0, 0}, {1, 1}, {1, 0}, 0.3));
    EXPECT_FALSE(cr::ControlUtils::objectVelocityAimedToPoint({0, 0}, {-1, -1}, {-1, 0}, 0.3));

    // velocity in same 'wrong' direction but with loose margins, so it should be true
    EXPECT_TRUE(cr::ControlUtils::objectVelocityAimedToPoint({0, 0}, {1, 1}, {1, 0}, rtt::toRadians(91)));
    EXPECT_TRUE(cr::ControlUtils::objectVelocityAimedToPoint({0, 0}, {-1, -1}, {-1, 0}, rtt::toRadians(91)));

    // the margin should be pretty strict
    EXPECT_FALSE(cr::ControlUtils::objectVelocityAimedToPoint({0, 0}, {1, 1}, {1, 0}, rtt::toRadians(90)));
    EXPECT_FALSE(cr::ControlUtils::objectVelocityAimedToPoint({0, 0}, {-1, -1}, {-1, 0}, rtt::toRadians(90)));
}