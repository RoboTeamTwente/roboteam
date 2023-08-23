//
// Created by alexander on 19-04-22.
//
#include <gtest/gtest.h>

#include <roboteam_utils/Field.hpp>

#include "helpers/FieldHelper.h"
#include "helpers/WorldHelper.h"
#include "world/FieldComputations.h"
#include "world/World.hpp"

TEST(FieldHelperTests, DefenseAreaTest) {
    using namespace rtt::ai;
    auto protoField = testhelpers::FieldHelper::generateField();
    auto protoWorld = testhelpers::WorldHelper::getWorldMsg(11, 11, true, protoField);
    auto const& [_, world] = rtt::world::World::instance();
    world->updateWorld(protoWorld);
    world->updateField(protoField);

    auto field = world->getField().value();
    auto fieldLength = field.playArea.width();
    auto defAreaDepth = field.leftDefenseArea.right() - field.leftGoalArea.rightLine().center().x;
    auto defAreaWidth = field.leftDefenseArea.top();

    EXPECT_TRUE(field.leftDefenseArea.contains(rtt::Vector2(-fieldLength * 0.49, 0)));
    EXPECT_TRUE(field.rightDefenseArea.contains(rtt::Vector2(fieldLength * 0.49, 0)));

    EXPECT_FALSE(field.rightDefenseArea.contains(rtt::Vector2(fieldLength * 0.5 - defAreaDepth * 1.01, 0)));
    EXPECT_FALSE(field.leftDefenseArea.contains(rtt::Vector2(fieldLength * 0.5 + defAreaDepth * 1.01, 0)));

    EXPECT_TRUE(field.leftDefenseArea.contains(rtt::Vector2(-fieldLength * 0.5 + defAreaDepth * 0.9, 0)));
    EXPECT_TRUE(field.rightDefenseArea.contains(rtt::Vector2(fieldLength * 0.5 - defAreaDepth * 0.9, 0)));

    EXPECT_TRUE(field.leftDefenseArea.contains(rtt::Vector2(-fieldLength * 0.5 + defAreaDepth * 0.5, defAreaWidth * 0.99)));
    EXPECT_TRUE(field.rightDefenseArea.contains(rtt::Vector2(fieldLength * 0.5 - defAreaDepth * 0.5, defAreaWidth * 0.99)));

    EXPECT_FALSE(field.leftDefenseArea.contains(rtt::Vector2(-fieldLength * 0.5 + defAreaDepth * 0.5, defAreaWidth * 1.01)));
    EXPECT_FALSE(field.rightDefenseArea.contains(rtt::Vector2(fieldLength * 0.5 - defAreaDepth * 0.5, defAreaWidth * 1.01)));
}

TEST(FieldHelperTests, FieldTest) {
    using namespace rtt::ai;
    auto protoField = testhelpers::FieldHelper::generateField();
    auto protoWorld = testhelpers::WorldHelper::getWorldMsg(11, 11, true, protoField);
    auto const& [_, world] = rtt::world::World::instance();
    world->updateWorld(protoWorld);
    world->updateField(protoField);

    auto field = world->getField().value();
    auto fieldWidth = field.playArea.width();
    auto fieldHeight = field.playArea.height();

    // TODO: What does this line do? Should it be tested? Or can it be removed?
    field.playArea.contains(rtt::Vector2(0.99 * fieldWidth, 0.99 * fieldHeight));
    EXPECT_TRUE(field.playArea.contains(rtt::Vector2(0.49 * fieldWidth, 0.49 * fieldHeight)));
    EXPECT_TRUE(field.playArea.contains(rtt::Vector2(-0.49 * fieldWidth, -0.49 * fieldHeight)));

    EXPECT_FALSE(field.playArea.contains(rtt::Vector2(0.51 * fieldWidth, 0.51 * fieldHeight)));
    EXPECT_FALSE(field.playArea.contains(rtt::Vector2(-0.51 * fieldWidth, -0.51 * fieldHeight)));
}
