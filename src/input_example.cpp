#include "ros/ros.h"

#include "roboteam_msgs/RobotCommand.h"

#include <vector>
#include <iostream>

int id;
bool active;
float x_vel;
float y_vel;
float w_vel;
bool dribbler;
float kick_vel;
float chip_vel;

int main(int argc, char **argv)
{
    // Create ros node 'input_example' and advertise on ropic 'robotcommands'
    ros::init(argc, argv, "input_example");
    ros::NodeHandle n;
    ros::Publisher chatter_pub = n.advertise<roboteam_msgs::RobotCommand>("robotcommands", 1000);
    ros::Rate loop_rate(60);
    
    // Keyboard inputs
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

    // Initialize RobotCommand message;
    roboteam_msgs::RobotCommand command;

    // Fill the RobotCommand message with the keyboard inputs
    command.id = id;
    command.active = active;
    command.x_vel = x_vel;
    command.y_vel = y_vel;
    command.w_vel = w_vel;
    command.dribbler = dribbler;
    command.kicker_vel = kick_vel;
    command.chipper_vel = chip_vel;

    // Publish the command
    chatter_pub.publish(command);

    ROS_INFO_STREAM("command sent for robot: " << command.id);

    loop_rate.sleep();
    ros::spin();

	return 0;
}
