/* 	 ______   _______  _______  ______     _______  _______  ______   _______
 *	(  __  \ (  ____ \(  ___  )(  __  \   (  ____ \(  ___  )(  __  \ (  ____ \
 *	| (  \  )| (    \/| (   ) || (  \  )  | (    \/| (   ) || (  \  )| (    \/
 *	| |   ) || (__    | (___) || |   ) |  | |      | |   | || |   ) || (__
 *	| |   | ||  __)   |  ___  || |   | |  | |      | |   | || |   | ||  __)
 *	| |   ) || (      | (   ) || |   ) |  | |      | |   | || |   ) || (
 *	| (__/  )| (____/\| )   ( || (__/  )  | (____/\| (___) || (__/  )| (____/\
 *	(______/ (_______/|/     \|(______/   (_______/(_______)(______/ (_______/
 *
 * This corresponding class that is being tested contains only dead code. Remove this tag if you use this code and make sure to remove this tag at other places as well that will
 * become alive by using this code.
 * Do not read/document/redesign/analyse/test/optimize/etc. any of this code, because it is a waste of your time! This code was not removed or placed at another branch, because
 * other software developers are very attached to this code and are afraid that this code might be used at some day (but I think it won't be used at all and should be removed).
 */

//
// Created by rolf on 28-01-20.
//

#include <gtest/gtest.h>
#include "roboteam_utils/Triangle.h"
#include "roboteam_utils/LineSegment.h"
#include "roboteam_utils/Line.h"
namespace rtt {
TEST(Triangle, basic) {
    Vector2 point1(1, 1);
    Vector2 point2(3, 1);
    Vector2 point3(2, 4);
    Triangle triangle(point1, point2, point3);
    EXPECT_EQ(point1, triangle.corner1);
    EXPECT_EQ(point2, triangle.corner2);
    EXPECT_EQ(point3, triangle.corner3);
    for (const auto &corner : triangle.corners()) {
        EXPECT_TRUE(corner == point1 || corner == point2 || corner == point3);
    }
    for (const auto &line : triangle.lines()) {
        EXPECT_TRUE(line.start == point1 || line.start == point2 || line.start == point3);
        EXPECT_TRUE(line.end == point1 || line.end == point2 || line.end == point3);
        EXPECT_TRUE(line.start != line.end);
    }
}

TEST(Triangle, area){
    Vector2 point1(1, 1);
    Vector2 point2(3, 1);
    Vector2 point3(2, 4);
    Triangle triangle(point1, point2, point3);
    EXPECT_DOUBLE_EQ(triangle.area(), 3.0);

    Vector2 point4(5,1);
    Triangle zeroArea(point1,point2,point4);
    EXPECT_DOUBLE_EQ(zeroArea.area(),0.0);
}
TEST(Triangle, contains){
    Vector2 point1(1, 1);
    Vector2 point2(3, 1);
    Vector2 point3(2, 4);
    Triangle triangle(point1, point2, point3);
    EXPECT_TRUE(triangle.contains(point1));
    EXPECT_TRUE(triangle.contains(point2));
    EXPECT_TRUE(triangle.contains(point3));
    EXPECT_TRUE(triangle.contains((point1+point2)*0.5));
    EXPECT_TRUE(triangle.contains((point2+point3)*0.5));
    EXPECT_TRUE(triangle.contains((point3+point1)*0.5));
    EXPECT_TRUE(triangle.contains((point1+point2+point3)/3));
    EXPECT_FALSE(triangle.contains(Vector2(1,0)));
}
//Can be better but this relies on line segment and line intersections which are very heavily tested so there is no need to overtest
TEST(Triangle,intersections){
    Vector2 point1(1, 1);
    Vector2 point2(3, 1);
    Vector2 point3(2, 4);
    Triangle triangle(point1, point2, point3);
    Line testLine(Vector2(1,2),Vector2(1,3));
    LineSegment segment(Vector2(1,2),Vector2(1,3));
    LineSegment second(Vector2(1,0),Vector2(1,3));
    EXPECT_TRUE(triangle.doesIntersect(testLine));
    EXPECT_FALSE(triangle.doesIntersect(segment));
    EXPECT_TRUE(triangle.doesIntersect(second));

    EXPECT_GT(triangle.intersects(testLine).size(),0);
    EXPECT_GT(triangle.intersects(second).size(),0);
    EXPECT_EQ(triangle.intersects(segment).size(),0);
}
}
