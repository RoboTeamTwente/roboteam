//
// Created by mrlukasbos on 7-3-19.
//

#include <gtest/gtest.h>
#include <ros/ros.h>

int main(int argc, char** argv) {
    testing::InitGoogleTest(&argc, argv);

    // create a ROS node for the tests
    ros::init(argc, argv, "tester");
    ros::NodeHandle nh;

    return RUN_ALL_TESTS();
}
