#include "roboteam_utils/Vector2.h"
#include <gtest/gtest.h>
#include <cmath>

using namespace rtt;

TEST(VectorTests, instantiation) {
    Vector2 def;
    Vector2 tenten(10, 10);
    Vector2 fivezero(5, 0);
    ASSERT_DOUBLE_EQ(def.x, 0);
    ASSERT_DOUBLE_EQ(def.y, 0);
    ASSERT_DOUBLE_EQ(tenten.x, 10);
    ASSERT_DOUBLE_EQ(tenten.y, 10);
    ASSERT_DOUBLE_EQ(fivezero.x, 5);
    ASSERT_DOUBLE_EQ(fivezero.y, 0);
}

TEST(VectorTests, operators) {
    Vector2 def;
    Vector2 tenten(10, 10);
    Vector2 fivezero(5, 0);
    ASSERT_TRUE(def == def);
    ASSERT_TRUE(tenten == tenten);
    ASSERT_TRUE(fivezero == fivezero);
    ASSERT_FALSE(def != def);
    ASSERT_FALSE(tenten != tenten);
    ASSERT_FALSE(fivezero != fivezero);
    Vector2 tmp = def + tenten;
    ASSERT_DOUBLE_EQ(tmp.x, 10);
    ASSERT_DOUBLE_EQ(tmp.y, 10);
    tmp = tenten - fivezero;
    ASSERT_DOUBLE_EQ(tmp.x, 5);
    ASSERT_DOUBLE_EQ(tmp.y, 10);
    tmp = tenten * fivezero;
    ASSERT_DOUBLE_EQ(tmp.x, 50);
    ASSERT_DOUBLE_EQ(tmp.y, 0);
}

#define SQRT200 14.1421356237309505

TEST(VectorTests, math) {
    Vector2 def;
    Vector2 tenten(10, 10);
    Vector2 fivezero(5, 0);
    
    ASSERT_DOUBLE_EQ(def.length(), 0);
    ASSERT_DOUBLE_EQ(tenten.length(), SQRT200);
    ASSERT_DOUBLE_EQ(fivezero.length(), 5);
    
    ASSERT_DOUBLE_EQ(def.normalize().x, 0);
    ASSERT_DOUBLE_EQ(def.normalize().y, 0);
    ASSERT_DOUBLE_EQ(tenten.normalize().x, 10 / SQRT200);
    ASSERT_DOUBLE_EQ(tenten.normalize().y, 10 / SQRT200);
    ASSERT_DOUBLE_EQ(fivezero.normalize().x, 1);
    ASSERT_DOUBLE_EQ(fivezero.normalize().y, 0);
    
    ASSERT_DOUBLE_EQ(def.scale(3.5).x, 0);
    ASSERT_DOUBLE_EQ(def.scale(3.5).y, 0);
    ASSERT_DOUBLE_EQ(tenten.scale(3.5).x, 35);
    ASSERT_DOUBLE_EQ(tenten.scale(3.5).y, 35);
    ASSERT_DOUBLE_EQ(fivezero.scale(3.5).x, 17.5);
    ASSERT_DOUBLE_EQ(fivezero.scale(3.5).y, 0);
    
    ASSERT_DOUBLE_EQ(def.dist(tenten), SQRT200);
    ASSERT_DOUBLE_EQ(tenten.dist(fivezero), 11.1803398874989485);
    
    ASSERT_DOUBLE_EQ(def.dot(tenten), 0);
    ASSERT_DOUBLE_EQ(tenten.dot(fivezero), 50);
    
    ASSERT_DOUBLE_EQ(M_PI_4l, tenten.angle());
    ASSERT_DOUBLE_EQ(0, fivezero.angle());
    
    Vector2 proj = fivezero.project(def, tenten);
    ASSERT_DOUBLE_EQ(2.5, proj.x);
    ASSERT_DOUBLE_EQ(2.5, proj.y);
    proj = tenten.project(def, fivezero);
    ASSERT_DOUBLE_EQ(5, proj.x);
    ASSERT_DOUBLE_EQ(0, proj.y);
}