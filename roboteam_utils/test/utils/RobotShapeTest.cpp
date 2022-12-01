//
// Created by rolf on 18-10-20.
//

#include <roboteam_utils/RobotShape.h>
#include <gtest/gtest.h>

#include <roboteam_utils/LineSegment.h>

using namespace rtt;

TEST(RobotShapeTest,constructors){
    double radius = 0.1;
    double centerToFront = 0.05;
    RobotShape shape(Vector2(0,0),centerToFront,radius,Angle(0.0));
    EXPECT_EQ(shape.pos(),Vector2(0,0));
    EXPECT_DOUBLE_EQ(shape.angle().getValue(),0.0);
    EXPECT_EQ(shape.centerOfKickerPos(),Vector2(centerToFront,0));
    LineSegment kicker = shape.kicker();
    double diff = sqrt(radius*radius-centerToFront*centerToFront);

    //These are very crucial that they are in the right order,
    //as many of the internal calculations in the class rely on these being in exactly this order.
    EXPECT_EQ(kicker.start,Vector2(centerToFront,-diff));
    EXPECT_EQ(kicker.end,Vector2(centerToFront,diff));

    //Check if rotation is applied correctly
    RobotShape shape2(Vector2(0,0),centerToFront,radius,Angle(1.0));
    EXPECT_EQ(shape2.pos(),Vector2(0,0));
    EXPECT_DOUBLE_EQ(shape2.angle().getValue(),1.0);
    EXPECT_DOUBLE_EQ(shape2.centerOfKickerPos().x,Vector2(centerToFront,0).rotate(1.0).x);
    EXPECT_DOUBLE_EQ(shape2.centerOfKickerPos().y,Vector2(centerToFront,0).rotate(1.0).y);

}
TEST(RobotShapeTest, inFrontOfDribbler){
    RobotShape shape(Vector2(0,0),0.05,0.1,Angle(0.0));
    //half plane should be defined by the line x= 0.05
    EXPECT_TRUE(shape.inFrontOfDribbler(Vector2(0.1,0.0)));
    EXPECT_FALSE(shape.inFrontOfDribbler(Vector2(0.05,0.0)));//Exactly on the line is defined as NOT being in front of the dribbler
    EXPECT_FALSE(shape.inFrontOfDribbler(Vector2(0.0,0.0)));
}

TEST(RobotShapeTest, move){
    RobotShape shape(Vector2(0,0),0.05,0.1,Angle(0.0));
    EXPECT_EQ(shape.pos(),Vector2(0,0));
    shape.move(Vector2(2,1));
    EXPECT_EQ(shape.pos(),Vector2(2,1));
}
TEST(RobotShapeTest, contains){
    RobotShape shape(Vector2(0,0),0.05,0.1,Angle(0.0));
    EXPECT_TRUE(shape.contains(Vector2(-0.1,0.0)));//on the boundary at the back
    EXPECT_TRUE(shape.contains(Vector2(-0.08,0.0)));//completely in the robot at the back
    EXPECT_TRUE(shape.contains(Vector2(0.05,0.0)));//Center of dribbler
    EXPECT_FALSE(shape.contains(Vector2(0.08,0.0)));//In front of dribbler but within circle
    EXPECT_FALSE(shape.contains(Vector2(0.1,0.0)));// on circle radius in front of dribbler
    EXPECT_FALSE(shape.contains(Vector2(1,1)));//Far outside of robot
}

//This test does NOT try to test the circle and line intersections functions that much,
//rather just the logic used in the shape itself
TEST(RobotShapeTest, lineIntersection){
    RobotShape shape(Vector2(0,0),0.05,0.1,Angle(0.0));;

    //0 circle intersections
    LineSegment outsideOfCircle(Vector2(0,1),Vector2(1,0));
    EXPECT_FALSE(shape.doesIntersect(outsideOfCircle));
    EXPECT_EQ(shape.intersects(outsideOfCircle).size(),0);

    LineSegment insideOfCircle(Vector2(-0.04,0),Vector2(0.04,0));
    EXPECT_FALSE(shape.doesIntersect(insideOfCircle));
    EXPECT_EQ(shape.intersects(insideOfCircle).size(),0);

    LineSegment insideCircleIntersectsDribbler(Vector2(-0.04,0),Vector2(0.06,0));
    EXPECT_TRUE(shape.doesIntersect(insideCircleIntersectsDribbler));
    EXPECT_EQ(shape.intersects(insideCircleIntersectsDribbler).size(),1);
    EXPECT_DOUBLE_EQ(shape.intersects(insideCircleIntersectsDribbler)[0].x,0.05);
    EXPECT_EQ(shape.intersects(insideCircleIntersectsDribbler)[0],Vector2(0.05,0));

    //1 circle intersection (e.g. one inside, one outside or tangent line)
    LineSegment touchCircleInFrontOfRobot(Vector2(0.1,0.2), Vector2(0.1,-0.2));
    EXPECT_FALSE(shape.doesIntersect(touchCircleInFrontOfRobot));
    EXPECT_EQ(shape.intersects(touchCircleInFrontOfRobot).size(),0);

    LineSegment tangentCircle(Vector2(-0.1,0.2),Vector2(-0.1,-0.2));
    EXPECT_TRUE(shape.doesIntersect(tangentCircle));
    EXPECT_EQ(shape.intersects(tangentCircle).size(),1);
    EXPECT_EQ(shape.intersects(tangentCircle)[0],Vector2(-0.1,0));

    LineSegment intersectCircleBack(Vector2(-0.05,0),Vector2(-0.15,0));
    EXPECT_TRUE(shape.doesIntersect(intersectCircleBack));
    EXPECT_EQ(shape.intersects(intersectCircleBack).size(),1);
    EXPECT_EQ(shape.intersects(intersectCircleBack)[0],Vector2(-0.1,0));

    LineSegment intersectCircleFront(Vector2(-0.05,0),Vector2(0.2,0));
    EXPECT_TRUE(shape.doesIntersect(intersectCircleFront));
    EXPECT_EQ(shape.intersects(intersectCircleFront).size(),1);
    EXPECT_EQ(shape.intersects(intersectCircleFront)[0],Vector2(0.05,0));

    Vector2 bottomDribbler = shape.kicker().start;
    Vector2 topDribbler = shape.kicker().end;
    LineSegment intersectCorner(Vector2(0,0),bottomDribbler*2);
    LineSegment intersectCorner2(Vector2(0,0),topDribbler*2);

    EXPECT_TRUE(shape.doesIntersect(intersectCorner2));
    //EXPECT_EQ(shape.intersects(intersectCorner2).size(),1); //TODO: fix multiple
    EXPECT_EQ(shape.intersects(intersectCorner2)[0],topDribbler);

    EXPECT_TRUE(shape.doesIntersect(intersectCorner));
    //EXPECT_EQ(shape.intersects(intersectCorner).size(),1); //TODO: fix multiple
    EXPECT_EQ(shape.intersects(intersectCorner)[0],bottomDribbler);

    //The following example actually shows a numeric precision problem with circle, that's why we need the epsilon call
    Vector2 touchBottomVec(-bottomDribbler.y-std::numeric_limits<double>::epsilon(),bottomDribbler.x);
    LineSegment touchCorner(bottomDribbler+touchBottomVec,bottomDribbler-touchBottomVec);
    EXPECT_TRUE(shape.doesIntersect(touchCorner));
    EXPECT_EQ(shape.intersects(touchCorner).size(),1);
    EXPECT_EQ(shape.intersects(touchCorner)[0],bottomDribbler);

    //2 circle intersections
    LineSegment inFrontOfRobot(Vector2(0.07,0.2), Vector2(0.07,-0.2));
    EXPECT_FALSE(shape.doesIntersect(inFrontOfRobot));
    EXPECT_EQ(shape.intersects(inFrontOfRobot).size(),0);

    LineSegment throughRobot(Vector2(0.0,0.2), Vector2(0.0,-0.2));
    EXPECT_TRUE(shape.doesIntersect(throughRobot));
    EXPECT_EQ(shape.intersects(throughRobot).size(),2);
    EXPECT_EQ(shape.intersects(throughRobot)[0],Vector2(0,0.1));
    EXPECT_EQ(shape.intersects(throughRobot)[1],Vector2(0,-0.1));

    LineSegment throughDribbler(Vector2(0.2,0.0), Vector2(-0.2,0.0));
    EXPECT_TRUE(shape.doesIntersect(throughDribbler));
    EXPECT_EQ(shape.intersects(throughDribbler).size(),2);
    EXPECT_EQ(shape.intersects(throughDribbler)[0],Vector2(0.05,0));
    EXPECT_EQ(shape.intersects(throughDribbler)[1],Vector2(-0.1,0));

    std::swap(throughDribbler.start,throughDribbler.end);
    EXPECT_TRUE(shape.doesIntersect(throughDribbler));
    EXPECT_EQ(shape.intersects(throughDribbler).size(),2);
    EXPECT_EQ(shape.intersects(throughDribbler)[0],Vector2(-0.1,0));
    EXPECT_EQ(shape.intersects(throughDribbler)[1],Vector2(0.05,0));

    Vector2 centerFront(0.1,0);
    Vector2 diff = centerFront-bottomDribbler;
    LineSegment throughCornerAndFront(bottomDribbler-diff*2,bottomDribbler+diff*2);
    EXPECT_TRUE(shape.doesIntersect(throughCornerAndFront));
    EXPECT_GE(shape.intersects(throughCornerAndFront).size(),1);
    EXPECT_EQ(shape.intersects(throughCornerAndFront)[0],bottomDribbler);

    diff = Vector2(-0.1,0)-bottomDribbler;

    LineSegment throughCornerAndBack(bottomDribbler-diff*2,bottomDribbler+diff*2);
    EXPECT_TRUE(shape.doesIntersect(throughCornerAndBack));
    EXPECT_EQ(shape.intersects(throughCornerAndBack).size(),2);
    EXPECT_EQ(shape.intersects(throughCornerAndBack)[0],bottomDribbler);
    EXPECT_EQ(shape.intersects(throughCornerAndBack)[1],Vector2(-0.1,0));

    LineSegment overlapDribbler(Vector2(0.05,0.2),Vector2(0.05,-0.2));
    EXPECT_TRUE(shape.doesIntersect(overlapDribbler));
    EXPECT_EQ(shape.intersects(overlapDribbler).size(),2);
    EXPECT_EQ(shape.intersects(overlapDribbler)[0], topDribbler);
    EXPECT_EQ(shape.intersects(overlapDribbler)[1], bottomDribbler);

    LineSegment partialDribbler(Vector2(0.05,0.0),Vector2(0.05,-0.2));
    EXPECT_TRUE(shape.doesIntersect(partialDribbler));
    EXPECT_EQ(shape.intersects(partialDribbler).size(),2);
    EXPECT_EQ(shape.intersects(partialDribbler)[0], partialDribbler.start);
    EXPECT_EQ(shape.intersects(partialDribbler)[1], bottomDribbler);
}