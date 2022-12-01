//
// Created by rolf on 26-09-20.
//

//
// Created by rolf on 21-04-20.
//

#include <gtest/gtest.h>
#include <roboteam_utils/containers/circular_buffer.h>
using namespace rtt::collections;
void access() {
    circular_buffer<double, 1> s;
    [[maybe_unused]] double x = s.at(2);
}
void constAccess() {
    const circular_buffer<double, 1> s;
    double x = s.at(2);
}
TEST(CircularBufferTest, atThrowTest) {
    EXPECT_ANY_THROW(access());
    EXPECT_ANY_THROW(constAccess());
}

TEST(CircularBufferTest, construct) {
    circular_buffer<int, 4> test;
    for (int kI = 0; kI < 5; ++kI) {
        test.push_back(kI * kI);
    }
    EXPECT_EQ(test.size(), 4);
    EXPECT_EQ(test.back(), 16);
    EXPECT_EQ(test.front(), 1);

    EXPECT_EQ(test.pop_front(), 1);
    EXPECT_EQ(test.size(), 3);
    EXPECT_EQ(test.max_size(), 4);
    EXPECT_EQ(test.front(), 4);
    EXPECT_EQ(test.back(), 16);
}
struct vec2d {
    double x, y;
};
TEST(CircularBufferTest, emplace_back) {
    circular_buffer<vec2d, 3> vecs;
    EXPECT_EQ(vecs.size(), 0);
    EXPECT_EQ(vecs.emplace_back(3.0, 4.0).y, 4.0);
    EXPECT_EQ(vecs.size(), 1);
    vec2d elem = {3.0, 4.0};
    EXPECT_EQ(vecs[0].x, elem.x);
    EXPECT_EQ(vecs[0].y, elem.y);
}

TEST(CircularBufferTest, sizes) {
    circular_buffer<int, 2> test;
    EXPECT_TRUE(test.empty());
    test.push_back(8);
    EXPECT_FALSE(test.empty());
    EXPECT_FALSE(test.full());
    test.push_back(9);
    EXPECT_TRUE(test.full());
    test.pop_front();
    EXPECT_FALSE(test.full());
    EXPECT_FALSE(test.empty());
    test.pop_front();
    EXPECT_TRUE(test.empty());
}

TEST(CircularBufferTest, at) {
    circular_buffer<int, 2> test;
    test.push_back(1);
    test.push_back(2);
    EXPECT_EQ(test.at(0), 1);
    EXPECT_EQ(test.at(1), 2);
    test.push_back(3);
    EXPECT_EQ(test.at(0), 2);
    EXPECT_EQ(test.at(1), 3);
}

TEST(CircularBufferTest, constness) {
    circular_buffer<int, 2> test;
    test.push_back(1);
    test.push_back(2);
    test.push_back(3);
    const auto x = test;

    EXPECT_EQ(x[0], 2);
    EXPECT_EQ(x[1], 3);
    EXPECT_EQ(x.back(), 3);
    EXPECT_EQ(x.front(), 2);
    EXPECT_EQ(x.at(1), 3);
    EXPECT_EQ(x.at(0), 2);
}

TEST(CircularBufferTest, push_back_const_ref) {
    circular_buffer<int, 2> test;
    int x = 4;
    const int& ref = x;
    test.push_back(ref);
    EXPECT_EQ(test[0], 4);
}