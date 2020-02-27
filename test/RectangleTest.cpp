//
// Created by rolf on 22-01-20.
//

#include <gtest/gtest.h>
#include <roboteam_utils/Rectangle.h>
#include <roboteam_utils/LineSegment.h>
namespace rtt{
static Rectangle rect(Vector2(-1,1),Vector2(1,2));
static Rectangle nullExample(Vector2(0,0),Vector2(0,0));
TEST(Rectangle, cohenCodes){
    EXPECT_EQ(rect.CohenSutherlandCode(Vector2(0,1.2)), 0x00);
    EXPECT_EQ(rect.CohenSutherlandCode(Vector2(-1,1)), 0x00);
    EXPECT_EQ(rect.CohenSutherlandCode(Vector2(-1.5,1.5)), 0x01);
    EXPECT_EQ(rect.CohenSutherlandCode(Vector2(1.5,1.2)), 0x02);
    EXPECT_EQ(rect.CohenSutherlandCode(Vector2(1.5,2)), 0x02);
    EXPECT_EQ(rect.CohenSutherlandCode(Vector2(-.5,0)), 0x04);
    EXPECT_EQ(rect.CohenSutherlandCode(Vector2(-1,0)), 0x04);
    EXPECT_EQ(rect.CohenSutherlandCode(Vector2(-1,3)), 0x08);
    EXPECT_EQ(rect.CohenSutherlandCode(Vector2(-.5,3)), 0x08);
    EXPECT_EQ(rect.CohenSutherlandCode(Vector2(-1.1,0)), 0x05);
    EXPECT_EQ(rect.CohenSutherlandCode(Vector2(1.1,0)), 0x06);
    EXPECT_EQ(rect.CohenSutherlandCode(Vector2(-1.5,2.5)), 0x09);
    EXPECT_EQ(rect.CohenSutherlandCode(Vector2(1.5,2.5)), 0x0A);
}

TEST(Rectangle, degenerateCohenCodes){
    EXPECT_EQ(nullExample.CohenSutherlandCode(Vector2(0,0)), 0x00);
    EXPECT_EQ(nullExample.CohenSutherlandCode(Vector2(1,0)), 0x02);
    EXPECT_EQ(nullExample.CohenSutherlandCode(Vector2(-1,0)), 0x01);
    EXPECT_EQ(nullExample.CohenSutherlandCode(Vector2(0,1)), 0x08);
    EXPECT_EQ(nullExample.CohenSutherlandCode(Vector2(0,-.1)), 0x04);
}
TEST(Rectangle , contains){
    EXPECT_TRUE(rect.contains(Vector2(0,1.5)));
    EXPECT_TRUE(rect.contains(Vector2(-1,1.5)));
    EXPECT_FALSE(rect.contains(Vector2(-2,1.5)));
    EXPECT_TRUE(rect.contains(Vector2(1,1.5)));
    EXPECT_FALSE(rect.contains(Vector2(2,1.5)));

    EXPECT_TRUE(rect.contains(Vector2(0,2)));
    EXPECT_TRUE(rect.contains(Vector2(0,1)));
    EXPECT_FALSE(rect.contains(Vector2(0,2.5)));
    EXPECT_FALSE(rect.contains(Vector2(0,0.5)));

    EXPECT_FALSE(rect.contains(Vector2(-2,2.5)));
    EXPECT_FALSE(rect.contains(Vector2(2,0.5)));
    EXPECT_FALSE(rect.contains(Vector2(2,2.5)));
    EXPECT_FALSE(rect.contains(Vector2(-2,0.5)));
}
TEST(Rectangle,segmentIntersection){
    auto results=nullExample.intersects(LineSegment(Vector2(1,0),Vector2(-1,0)));
    EXPECT_FALSE(results.empty());
    results = nullExample.intersects(LineSegment(Vector2(1,.1),Vector2(-1,0)));
    EXPECT_TRUE(results.empty());
    Vector2 v1(-1,1);
    Vector2 v2(1,2);
    auto res = rect.intersects(LineSegment(Vector2(2,1.5),Vector2(-2,1.5)));
    EXPECT_FALSE(res.empty());
    res = rect.intersects(LineSegment(Vector2(-2,.5),Vector2(2,2.5)));
    EXPECT_FALSE(res.empty());
    EXPECT_TRUE(res.size() == 2);
    for (const auto& point : res){
        EXPECT_TRUE(point==v1 || point==v2 );
    }
}
}
