//
// Created by rolf on 26-09-20.
//

#include <gtest/gtest.h>
#include <roboteam_utils/containers/static_vector.hpp>
using namespace rtt::collections;
TEST(static_vector,initialisation){
    static_vector<double,3> x;
    static_vector<int,2> y = {3,4};
    EXPECT_EQ(y[0],3);
    EXPECT_EQ(y[1],4);
}

TEST(static_vector,simplePushBack){
    static_vector<double,2> x;
    x.push_back(3.0); //move
    EXPECT_EQ(3.0,x[0]);

    const double & y = 9.0;
    x.push_back(y); // const ref
    EXPECT_EQ(x[1],9.0);
    EXPECT_THROW(x.push_back(2.1),std::runtime_error);
}
TEST(static_vector,erase){
    static_vector<double,3> x;
    x.push_back(3.0);
    EXPECT_EQ(x.back(),3.0);
    x.push_back(4.0);
    EXPECT_EQ(x.back(),4.0);
    x.push_back(5.0);
    EXPECT_EQ(x.back(),5.0);

    x.erase(x.begin()+1);


}

TEST(static_vector,brackets){
    static_vector<double,4> x;
    x[3] = 0.1;
    EXPECT_EQ(0.1,x[3]);
}

TEST(static_vector, range_based_for_loop){
    static_vector<double,3> x;
    x.push_back(3.0);
    x.push_back(4.0);
    double sum = 0.0;
    for (const auto& elem : x){
        sum += elem;
    }
    EXPECT_EQ(sum,7.0);
    x.push_back(5.0);
    double sum2 = 0.0;
    for (auto& elem : x){
        sum2 += elem;
    }
    EXPECT_EQ(sum2,12.0);

    double sum3 = 0.0;
    for (auto elem : x){
        sum3+=elem;
    }
    EXPECT_EQ(sum3,12.0);
}
