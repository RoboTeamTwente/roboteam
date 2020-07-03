//
// Created by rolf on 24-02-20.
//

#include <gtest/gtest.h>
#include "roboteam_utils/Tube.h"
#include "roboteam_utils/Circle.h"

namespace rtt{

TEST(Tube,basic){
    Tube unitCircle;
    EXPECT_DOUBLE_EQ(unitCircle.radius,1.0);
    EXPECT_DOUBLE_EQ(unitCircle.lineSegment.length(),0.0);
    EXPECT_DOUBLE_EQ(unitCircle.lineSegment.start.x,0.0);
    EXPECT_DOUBLE_EQ(unitCircle.lineSegment.start.y,0.0);
    EXPECT_DOUBLE_EQ(unitCircle.lineSegment.end.x,0.0);
    EXPECT_DOUBLE_EQ(unitCircle.lineSegment.end.y,0.0);
    EXPECT_TRUE(unitCircle.isCircle());
    Vector2 start(0,0);
    Vector2 end(2,2);
    double radius = 0.5;
    Tube case1(start,end,radius);
    EXPECT_DOUBLE_EQ(case1.lineSegment.start.x,start.x);
    EXPECT_DOUBLE_EQ(case1.lineSegment.start.y,start.y);
    EXPECT_DOUBLE_EQ(case1.lineSegment.end.x,end.x);
    EXPECT_DOUBLE_EQ(case1.lineSegment.end.y,end.y);
    EXPECT_DOUBLE_EQ(case1.radius,radius);

    LineSegment line(Vector2(0,1),Vector2(3,4));
    double secondRadius = 5;
    Tube case2(line,secondRadius);

    EXPECT_DOUBLE_EQ(case2.lineSegment.start.x,line.start.x);
    EXPECT_DOUBLE_EQ(case2.lineSegment.start.y,line.start.y);
    EXPECT_DOUBLE_EQ(case2.lineSegment.end.x,line.end.x);
    EXPECT_DOUBLE_EQ(case2.lineSegment.end.y,line.end.y);
    EXPECT_DOUBLE_EQ(case2.radius,secondRadius);
}
TEST(Tube,contains){
    Vector2 start(1,1);
    Vector2 end(9,1);
    double radius = 1.0;
    Tube tube(start,end,radius);
    //some basic sanity checks...
    EXPECT_TRUE(tube.contains(start));
    EXPECT_TRUE(tube.contains(end));
    Vector2 test1(1,0); //test if the boundaries are included as well.
    Vector2 test2(0,1);
    EXPECT_TRUE(tube.contains(test1));
    EXPECT_TRUE(tube.contains(test2));
    Vector2 test3(0,0); // is sqrt2 away so should not be in tube (strictly outside
    EXPECT_FALSE(tube.contains(test3));
    Vector2 test4(5,1.5);//is strictly inside the tube
    EXPECT_TRUE(tube.contains(test4));
    Vector2 test5(5,2.5); //last point, should be strictly outside
    EXPECT_FALSE(tube.contains(test5));
}

TEST(Tube,circleIntersect){
    Vector2 start(1,1);
    Vector2 end(9,1);
    double radius = 1.0;
    Tube tube(start,end,radius);

    Circle circle(Vector2(0,0),1);
    EXPECT_TRUE(tube.doesIntersectOrContain(circle));

    Circle circle2(Vector2(0,0),sqrt(2)-1);
    Circle circle3(Vector2(0,0),0.1);
    EXPECT_TRUE(tube.doesIntersectOrContain(circle2)); //test if boundary intersection works, exactly touches the tube
    EXPECT_FALSE(tube.doesIntersectOrContain(circle3));
}

/*
TEST(Tube,doesIntersect){
    Vector2 start(1,1);
    Vector2 end(9,1);
    double radius = 1.0;
    Tube tube(start,end,radius);
    LineSegment test1(Vector2(5,-3),Vector2(5,3)); // actually intersects the line segment of the tube
    LineSegment test2(Vector2(4,1.5),Vector2(6,1.5)); // parallel, but inside the tube
    LineSegment test3(Vector2(4,3),Vector2(5,3)); // parallel but outside tube
    LineSegment test4(Vector2(0.5,3),Vector2(0.5,-3));// Intersects line segment around cap
    LineSegment test5(Vector2(9.5,-3),Vector2(9.5,3)); //similar as above
    LineSegment test6(Vector2(0,0),Vector2(0,2)); //only barely touches the tube at (1,0)
    EXPECT_TRUE(tube.doesIntersectOrContain(test1));
    EXPECT_TRUE(tube.doesIntersectOrContain(test2));
    EXPECT_FALSE(tube.doesIntersectOrContain(test3));
    EXPECT_TRUE(tube.doesIntersectOrContain(test4));
    EXPECT_TRUE(tube.doesIntersectOrContain(test5));
    EXPECT_TRUE(tube.doesIntersectOrContain(test6));
}
 */
}

