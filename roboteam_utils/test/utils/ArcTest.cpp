#include <gtest/gtest.h>

#include "roboteam_utils/Arc.h"

namespace rtt {

TEST(ArcTests, unitCircle) {
    Arc circle;
    ASSERT_EQ(circle.length, circle.width);
    ASSERT_DOUBLE_EQ(1.0, circle.length);
    ASSERT_DOUBLE_EQ(2 * M_PI - std::numeric_limits<float>::epsilon(), circle.angleEnd - circle.angleStart);

    // this is a circle
    ASSERT_TRUE(circle.isCircle());
    ASSERT_TRUE(circle.isPartialCircle());

    // all angles are within the circle
    ASSERT_TRUE(circle.angleWithinArc(1.0));
    ASSERT_TRUE(circle.angleWithinArc(5.0));
    ASSERT_TRUE(circle.angleWithinArc(-1.0));

    // test point incidence
    Vector2 in(0.5, 0.0);
    ASSERT_TRUE(circle.pointInArc(in));
    ASSERT_FALSE(circle.pointOnArc(in));

    Vector2 on(0.0, 1.0);
    ASSERT_TRUE(circle.pointOnArc(on));
    ASSERT_TRUE(circle.pointInArc(on));

    Vector2 out(2.0, 2.0);
    ASSERT_FALSE(circle.pointInArc(out));
    ASSERT_FALSE(circle.pointOnArc(out));

    // test line intersections
    Vector2 lineA1(-2.0, 0.0), lineA2(2.0, 0.0);  // two intersections
    auto isect = circle.intersectionWithLine(lineA1, lineA2);
    ASSERT_TRUE((bool)isect.first);
    ASSERT_TRUE((bool)isect.second);
    ASSERT_DOUBLE_EQ(1.0, isect.first->x);
    ASSERT_DOUBLE_EQ(-1.0, isect.second->x);
    ASSERT_DOUBLE_EQ(0.0, isect.first->y);
    ASSERT_DOUBLE_EQ(0.0, isect.second->y);

    Vector2 lineB1(-1.0, -1.0), lineB2(-1.0, 1.0);  // one intersection
    isect = circle.intersectionWithLine(lineB1, lineB2);
    ASSERT_TRUE((bool)isect.first);
    ASSERT_FALSE((bool)isect.second);
    ASSERT_DOUBLE_EQ(-1.0, isect.first->x);
    ASSERT_DOUBLE_EQ(0.0, isect.first->y);
}

TEST(ArcTests, nonUnitCircle) {
    // Test with a non-unit circle
    Arc nonUnitCircle(Vector2(0, 0), 2.0, 0.0, M_PI);
    ASSERT_EQ(nonUnitCircle.length, nonUnitCircle.width);
    ASSERT_DOUBLE_EQ(2.0, nonUnitCircle.length);
    ASSERT_DOUBLE_EQ(M_PI, nonUnitCircle.angleEnd - nonUnitCircle.angleStart);
    ASSERT_FALSE(nonUnitCircle.isCircle());
    ASSERT_TRUE(nonUnitCircle.isPartialCircle());

    // Test angleWithinArc with a non-unit circle
    ASSERT_TRUE(nonUnitCircle.angleWithinArc(1.0));
    ASSERT_FALSE(nonUnitCircle.angleWithinArc(5.0));
    ASSERT_FALSE(nonUnitCircle.angleWithinArc(-1.0));

    // Test pointInArc and pointOnArc with a non-unit circle
    Vector2 in(1.0, 0.0);
    ASSERT_TRUE(nonUnitCircle.pointInArc(in));
    ASSERT_FALSE(nonUnitCircle.pointOnArc(in));

    Vector2 on(0.0, 2.0);
    ASSERT_TRUE(nonUnitCircle.pointOnArc(on));
    ASSERT_TRUE(nonUnitCircle.pointInArc(on));

    Vector2 out(3.0, 3.0);
    ASSERT_FALSE(nonUnitCircle.pointInArc(out));
    ASSERT_FALSE(nonUnitCircle.pointOnArc(out));

    // Test intersectionWithLine with a non-unit circle
    Vector2 lineA1(-3.0, 0.0), lineA2(3.0, 0.0);  // two intersections
    auto isect = nonUnitCircle.intersectionWithLine(lineA1, lineA2);
    ASSERT_TRUE((bool)isect.first);
    ASSERT_TRUE((bool)isect.second);
    ASSERT_DOUBLE_EQ(2.0, isect.first->x);
    ASSERT_DOUBLE_EQ(-2.0, isect.second->x);
    ASSERT_DOUBLE_EQ(0.0, isect.first->y);
    ASSERT_DOUBLE_EQ(0.0, isect.second->y);

    Vector2 lineB1(-2.0, -2.0), lineB2(-2.0, 2.0);  // one intersection
    isect = nonUnitCircle.intersectionWithLine(lineB1, lineB2);
    ASSERT_TRUE((bool)isect.first);
    ASSERT_FALSE((bool)isect.second);
    ASSERT_DOUBLE_EQ(-2.0, isect.first->x);
    ASSERT_DOUBLE_EQ(0.0, isect.first->y);
}

}  // namespace rtt