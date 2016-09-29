#include "roboteam_utils/Position.h"
#include <gtest/gtest.h>

using namespace roboteam_utils;

//#define ASSERT_ALL_EQ(p, x, y, rot) ASSERT_DOUBLE_EQ(p.x, x); \
//                                    ASSERT_DOUBLE_EQ(p.y, y); \
//                                    ASSERT_DOUBLE_EQ(p.rot, rot);
void ASSERT_ALL_EQ(Position p, double x, double y, double rot) {
    ASSERT_DOUBLE_EQ(p.x, x);
    ASSERT_DOUBLE_EQ(p.y, y);
    ASSERT_DOUBLE_EQ(p.rot, rot);
}

TEST(PositionTests, instantiation) {
    Position def;
    Position pos2(5, 10, 0);
    Position pos3(42, 1.414, 3.1415);
    ASSERT_ALL_EQ(def, 0, 0, 0);
    ASSERT_ALL_EQ(pos2, 5, 10, 0);
    ASSERT_ALL_EQ(pos3, 42, 1.414, 3.1415);
}

TEST(PositionTests, equality) {
    Position def;
    Position pos2(5, 10, 0);
    Position pos3(42, 1.414, 3.1415);
    ASSERT_TRUE(def == def);
    ASSERT_TRUE(pos2 == pos2);
    ASSERT_TRUE(pos3 == pos3);
    ASSERT_FALSE(def != def);
    ASSERT_FALSE(pos2 != pos2);
    ASSERT_FALSE(pos3 != pos3);
    ASSERT_TRUE(def != pos2);
    ASSERT_TRUE(pos3 != pos2);
}

TEST(PositionTests, ops) {
    Position def;
    Position pos2(5, 10, 0);
    Position pos3(42, 1.414, 3.1415);
    ASSERT_ALL_EQ(def.rotate(4), 0, 0, 4);
    ASSERT_ALL_EQ(pos3.rotate(-2), 42, 1.414, 1.1415);
    Vector2 v1(5, 10);
    Vector2 v2(-5, -1);
    ASSERT_ALL_EQ(pos2.translate(v1), 10, 20, 0);
    ASSERT_ALL_EQ(pos3.translate(v2), 37, 0.414, 3.1415);
    ASSERT_ALL_EQ(def.move(v1, 3.1415), 5, 10, 3.1415);
    ASSERT_ALL_EQ(pos3.move(v2, -3.1415), 37, 0.414, 0);
}