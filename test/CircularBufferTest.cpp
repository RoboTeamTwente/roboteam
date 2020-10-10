//
// Created by rolf on 26-09-20.
//

//
// Created by rolf on 21-04-20.
//

#include <gtest/gtest.h>
#include <roboteam_utils/containers/circular_buffer.h>
using namespace rtt::collections;
void access(){
    circular_buffer<double,1> s;
    double x = s.at(2);
}
TEST(CircularBufferTest,aTest){
    EXPECT_ANY_THROW(access());
}

TEST(CircularBufferTest,construct){
    circular_buffer<int,4> test;
    for (int kI = 0; kI < 5; ++kI) {
        test.push_back(kI*kI);
    }
    EXPECT_EQ(test.size(),4);
    EXPECT_EQ(test.back(),16);
    EXPECT_EQ(test.front(),1);

    EXPECT_EQ(test.pop_front(),1);
    EXPECT_EQ(test.size(),3);
    EXPECT_EQ(test.max_size(),4);
    EXPECT_EQ(test.front(),4);
    EXPECT_EQ(test.back(),16);
}
struct vec2d{
    double x,y;
};
TEST(CircularBufferTest,emplace_back){
    circular_buffer<vec2d,3> vecs;
    EXPECT_EQ(vecs.size(),0);
    EXPECT_EQ(vecs.emplace_back(3.0,4.0).y,4.0);
    EXPECT_EQ(vecs.size(),1);
    vec2d elem = {3.0,4.0};
    EXPECT_EQ(vecs[0].x,elem.x);
    EXPECT_EQ(vecs[0].y,elem.y);

}