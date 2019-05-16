//
// Created by rolf on 14-5-19.
//

#include "roboteam_utils/Polygon.h"
#include "roboteam_utils/Vector2.h"
#include <gtest/gtest.h>
using namespace rtt;
TEST(constructors, PolygonTest) {
    Vector2 leftCorner(1.0, 1.0), rightBottom(3.0, 1.0), rightTop(3.0, 4.0), leftTop(1.0, 4.0);
    Polygon rect(leftCorner, 2.0, 3.0);
    Polygon rect2({leftCorner, rightBottom, rightTop, leftTop});
    ASSERT_EQ(rect[0], leftCorner);
    ASSERT_EQ(rect[1], rightBottom);
    ASSERT_EQ(rect[2], rightTop);
    ASSERT_EQ(rect[3], leftTop);

    EXPECT_EQ(rect[0], rect2[0]);
    EXPECT_EQ(rect[1], rect2[1]);
    EXPECT_EQ(rect[2], rect2[2]);
    EXPECT_EQ(rect[3], rect2[3]);
}

TEST(basicFunctions, PolygonTest) {
    Vector2 leftCorner(1.0, 1.0), rightBottom(3.0, 1.0), rightTop(3.0, 4.0), leftTop(1.0, 4.0);
    Polygon rect(leftCorner, 2.0, 3.0);
    Polygon rect2({leftCorner, rightBottom, rightTop, leftTop});
    Polygon triangle({leftCorner, rightBottom, leftTop});

    std::vector<Polygon> objects = {rect, rect2, triangle};
    ASSERT_EQ(rect.amountOfVertices(), 4);
    ASSERT_EQ(rect2.amountOfVertices(), 4);
    ASSERT_EQ(triangle.amountOfVertices(), 3);

    for (const Polygon &obj:objects) {
        for (int i = 0; i < obj.amountOfVertices(); ++ i) {
            EXPECT_EQ(obj[i], obj.vertices[i]);
        }
    }
    for (const Polygon &obj:objects) {
        Polygon copy = obj, copy2 = obj;
        Vector2 moveBy(- 3.7, 2.5);
        copy.move(Vector2(0.0, 0.0));
        copy2.move(moveBy);
        for (int i = 0; i < obj.amountOfVertices(); ++ i) {
            EXPECT_EQ(copy[i], obj[i]);
            EXPECT_EQ(copy2[i], obj[i] + moveBy);
        }
    }
}
TEST(isSimple, PolygonTest) {
    Vector2 leftCorner(1.0, 1.0), rightBottom(3.0, 1.0), rightTop(3.0, 4.0), leftTop(1.0, 4.0);
    //should be simple
    Polygon rect(leftCorner, 2.0, 3.0);
    Polygon rect2({leftCorner, rightBottom, rightTop, leftTop});
    Polygon rect3({leftCorner, leftTop, rightTop, rightBottom});

    Polygon triangle({leftCorner, rightBottom, leftTop});

    EXPECT_TRUE(rect.isSimple());
    EXPECT_TRUE(rect2.isSimple());
    EXPECT_TRUE(rect3.isSimple());
    EXPECT_TRUE(triangle.isSimple());
    // all non-simple cases
    Polygon rect4({leftCorner, rightBottom, leftTop, rightTop});
    Polygon rect5({leftCorner, leftTop, rightBottom, rightTop});
    Polygon rect6({leftCorner, rightTop, leftTop, rightBottom});
    Polygon rect7({leftCorner, rightTop, rightBottom, leftTop});
    EXPECT_FALSE(rect4.isSimple());
    EXPECT_FALSE(rect5.isSimple());
    EXPECT_FALSE(rect6.isSimple());
    EXPECT_FALSE(rect7.isSimple());
}
TEST(isConvex, PolygonTest) {
    Vector2 leftCorner(1.0, 1.0), rightBottom(3.0, 1.0), rightTop(3.0, 4.0), leftTop(1.0, 4.0), convexPoint(4.0,
            2.0), nonConvexPoint(2.0, 2.0);
    Polygon convexPentagon({leftCorner, rightBottom, convexPoint, rightTop, leftTop});
    Polygon convexPentagonCCW({leftCorner, leftTop, rightTop, convexPoint, rightBottom});
    Polygon nonConvexPentagon({leftCorner, rightBottom, nonConvexPoint, rightTop, leftTop});
    Polygon nonConvexPentagonCCW({leftCorner, leftTop, rightTop, nonConvexPoint, rightBottom});
    // edge cases
    Vector2 convexPointLast(0.0, 2.0), nonConvexPointLast(0.0, 0.0), nonConvexPointLast2(2.0, 2.0);
    Polygon P1Last({leftCorner, rightBottom, rightTop, leftTop, convexPointLast});
    Polygon P1First({convexPointLast, leftCorner, rightBottom, rightTop, leftTop});
    Polygon P2Last({leftCorner, rightBottom, rightTop, leftTop, nonConvexPointLast});
    Polygon P2First({nonConvexPointLast, leftCorner, rightBottom, rightTop, leftTop});
    Polygon P3Last({leftCorner, rightBottom, rightTop, leftTop, nonConvexPointLast2});
    Polygon P3First({nonConvexPointLast2, leftCorner, rightBottom, rightTop, leftTop});
    Polygon P4({leftCorner, Vector2(2.0, 1.0), rightBottom, Vector2(3.0, 2.0), rightTop, Vector2(2.0, 4.0), leftTop,
                Vector2(1.0, 2.0)});

    std::vector<Polygon> allExamples = {convexPentagon, convexPentagonCCW, nonConvexPentagon, nonConvexPentagonCCW,
                                        P1Last, P1First, P2First, P2Last, P3First, P3Last, P4};
    //P1Last,P1First,;
    // convex calculation only makes sense if the examples we use are actually valid
    for (const Polygon &example : allExamples) {
        ASSERT_TRUE(example.isSimple());
    }
    EXPECT_TRUE(convexPentagon.isConvex());
    EXPECT_TRUE(convexPentagonCCW.isConvex());
    EXPECT_FALSE(nonConvexPentagon.isConvex());
    EXPECT_FALSE(nonConvexPentagonCCW.isConvex());

    EXPECT_TRUE(P1First.isConvex());
    EXPECT_TRUE(P1Last.isConvex());
    EXPECT_FALSE(P2First.isConvex());
    EXPECT_FALSE(P2Last.isConvex());
    EXPECT_FALSE(P3First.isConvex());
    EXPECT_FALSE(P3Last.isConvex());
    EXPECT_TRUE(P4.isConvex());

}
TEST(boundaryTests, PolygonTest) {
    Vector2 leftCorner(1.0, 1.0), rightBottom(3.0, 1.0), rightTop(3.0, 4.0), leftTop(1.0, 4.0);
    double width = 2.0;
    double height = 3.0;
    Polygon rect(leftCorner, width, height);
    Polygon rect2({leftCorner, rightBottom, rightTop, leftTop});
    Polygon triangle({leftCorner, rightBottom, leftTop});
    std::vector<Polygon> objects;
    for (const Polygon &obj:objects) {
        std::vector<LineSegment> lines = obj.getBoundary();
        for (int i = 0; i < lines.size(); ++ i) {
            EXPECT_EQ(lines[i].start, obj[i]);
            EXPECT_EQ(lines[i].end, i == lines.size() - 1 ? obj[0] : obj[i + 1]);
        }
        for (const LineSegment &line: lines) {
            EXPECT_TRUE(obj.isOnBoundary(line.start));
            EXPECT_TRUE(obj.isOnBoundary(line.end));
            EXPECT_TRUE(obj.isOnBoundary((line.start + line.end)*0.5));
        }
        double lenSum = 0;
        for (const LineSegment &line: lines) {
            lenSum += line.length();
        }
        EXPECT_EQ(lenSum, obj.perimeterLength());
    }
    EXPECT_EQ(rect.perimeterLength(), 2*(width + height));
    EXPECT_EQ(rect2.perimeterLength(), 2*(width + height));
    EXPECT_EQ(triangle.perimeterLength(), 2.0 + sqrt(13) + 3.0);

}
TEST(areas, PolygonTest) {
    double width = 4.0;
    double height = 4.0;
    double sideOffset = 1.0;
    Vector2 A(0.0, 0.0), B(width, 0.0), C(width + sideOffset, height), D(sideOffset, height), E(width, height), F(0.0,
            height);
    Polygon parallelogram({A, B, C, D});
    Polygon rectangle({A, B, E, F});
    Polygon triangleA({A, B, C}), triangleB({A, B, D}), triangleC({A, B, E}), triangleD({A, B, F});
    ASSERT_TRUE(parallelogram.isSimple());
    ASSERT_TRUE(rectangle.isSimple());
    EXPECT_EQ(parallelogram.area(), width*height);
    EXPECT_EQ(rectangle.area(), width*height);
    EXPECT_EQ(triangleA.area(), width*height*0.5);
    EXPECT_EQ(triangleB.area(), width*height*0.5);
    EXPECT_EQ(triangleC.area(),width*height*0.5);
    EXPECT_EQ(triangleD.area(),width*height*0.5);
    // below should hold for any random 3 points (could randomly generate some later perhaps)
    Vector2 G(-3.57,0.4);
    Polygon triangleE({A,D,G});
    Line l(A,D);
    EXPECT_EQ(triangleE.area(),0.5*l.length()*l.distanceToLine(G));
}
TEST(intersections, PolygonTest) {
    Vector2 leftCorner(1.0, 1.0), rightBottom(3.0, 1.0), rightTop(3.0, 3.0), leftTop(1.0, 3.0);
    Vector2 OP1(0.5,2.0),OP2(2.0,0.5),OP3(3.5,2.0),OP4(2.0,3.5);
    LineSegment L1(OP1,OP2), L2(OP2,OP3),L3(OP3,OP4),L4(OP4,OP1);
    Polygon rect({leftCorner, rightBottom, rightTop, leftTop});
    rect.intersections(L1);
    rect.intersections(L2);
    rect.intersections(L3);
    rect.intersections(L4);
    EXPECT_TRUE(rect.doesIntersect(L1));
    EXPECT_TRUE(rect.doesIntersect(L2));
    EXPECT_TRUE(rect.doesIntersect(L3));
    EXPECT_TRUE(rect.doesIntersect(L4));
    EXPECT_EQ(rect.intersections(L1).size(),2);
    EXPECT_EQ(rect.intersections(L2).size(),2);
    EXPECT_EQ(rect.intersections(L3).size(),2);
    EXPECT_EQ(rect.intersections(L4).size(),2);
    Vector2 OP5(2.0,2.0);
    LineSegment L5(OP1,OP5);
    EXPECT_TRUE(rect.doesIntersect(L5));
    EXPECT_EQ(rect.intersections(L5).size(),1);

    // edge cases
    Vector2 OPE1(0.0,2.0),OPE2(2.0,0.0),OPE3(4.0,2.0),OPE4(2.0,4.0);
    LineSegment LE1(OPE1,OPE2), LE2(OPE2,OPE3),LE3(OPE3,OPE4),LE4(OPE4,OPE1);
    EXPECT_TRUE(rect.doesIntersect(LE1));
    EXPECT_TRUE(rect.doesIntersect(LE2));
    EXPECT_TRUE(rect.doesIntersect(LE3));
    EXPECT_TRUE(rect.doesIntersect(LE4));
    ASSERT_EQ(rect.intersections(LE1).size(),1);
    ASSERT_EQ(rect.intersections(LE2).size(),1);
    ASSERT_EQ(rect.intersections(LE3).size(),1);
    ASSERT_EQ(rect.intersections(LE4).size(),1);
    EXPECT_EQ(rect.intersections(LE1)[0],leftCorner);
    EXPECT_EQ(rect.intersections(LE2)[0],rightBottom);
    EXPECT_EQ(rect.intersections(LE3)[0],rightTop);
    EXPECT_EQ(rect.intersections(LE4)[0],leftTop);

}
TEST(contains, PolygonTest) {
    Vector2 leftCorner(1.0, 1.0), rightBottom(3.0, 1.0), rightTop(3.0, 4.0), leftTop(1.0, 4.0);
    Polygon rect({leftCorner, rightBottom, rightTop, leftTop});
    Polygon triangle({leftCorner, rightBottom, leftTop});

    Vector2 P1(2.0,2.0),P2(2.5,2.5),P3(0.0,2.0),P4(0.1,0.1),P5(4.0,2.0),P6(2.0,6.0),P7(2.0,0.0);
    ASSERT_TRUE(rect.contains(P1));
    ASSERT_TRUE(rect.contains(P2));
    ASSERT_FALSE(rect.contains(P3));
    ASSERT_FALSE(rect.contains(P4));
    ASSERT_FALSE(rect.contains(P5));
    ASSERT_FALSE(rect.contains(P6));
    ASSERT_FALSE(rect.contains(P7));

    ASSERT_TRUE(triangle.contains(P1));
    ASSERT_FALSE(triangle.contains(P2));
    ASSERT_FALSE(triangle.contains(P3));
    ASSERT_FALSE(triangle.contains(P4));
    ASSERT_FALSE(triangle.contains(P5));
    ASSERT_FALSE(triangle.contains(P6));
    ASSERT_FALSE(triangle.contains(P7));

}
TEST(centroid, PolygonTest) {
    Vector2 A(5.0,3.0),B(6.0,1.0),C(7.0,8.0),D(1.0,2.0);
    Polygon triangle({A,B,C}),triangle2({A,B,D}),quadrilateral({A,B,C,D});
    Vector2 centroid1(6,4),centroid2(4.0,2.0),centroid3(4+8.0/9.0,4+2.0/9.0);
    ASSERT_EQ(triangle.centroid(),centroid1);
    ASSERT_EQ(triangle2.centroid(),centroid2);
    ASSERT_TRUE(quadrilateral.isSimple());
    ASSERT_EQ(quadrilateral.centroid(),centroid3);
}