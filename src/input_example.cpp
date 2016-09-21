#include "ros/ros.h"

#include "roboteam_robothub/robot_command.h"

#include <vector>
#include <iostream>

int main(int argc, char **argv)
{
    ros::init(argc, argv, "input_example");
    ros::NodeHandle n;
    ros::Publisher chatter_pub = n.advertise<roboteam_robothub::robot_command>("robotcommands", 1000);
    ros::Rate loop_rate(60);

    int id;
    bool active;
    float x_vel;
    float y_vel;
    float w_vel;
    bool dribbler;
    float kick_vel;
    float chip_vel;
    
    std::cout << "Enter a robot ID: ";
    std::cin >> id;
    std::cout << "Enter an x-velocity: ";
    std::cin >> x_vel;
    std::cout << "Enter a y-velocity: ";
    std::cin >> y_vel;
    std::cout << "Enter an angular velocity: ";
    std::cin >> w_vel;

    active = true;
    dribbler = false;
    kick_vel = 0.0;
    chip_vel = 0.0;

    roboteam_robothub::robot_command command;

    command.id = id;
    command.active = active;
    command.x_vel = x_vel;
    command.y_vel = y_vel;
    command.w_vel = w_vel;
    command.dribbler = dribbler;
    command.kick_vel = kick_vel;
    command.chip_vel = chip_vel;

    ROS_INFO_STREAM("command sent for robot: " << command.id);

    chatter_pub.publish(command);
    loop_rate.sleep();
    ros::spin();

  return 0;
}