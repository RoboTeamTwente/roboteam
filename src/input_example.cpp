#include "ros/ros.h"

#include "roboteam_msgs/RobotCommand.h"
#include "std_msgs/Float64MultiArray.h"
#include "std_msgs/MultiArrayDimension.h"

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

// float wheel1;
// float wheel2;
// float wheel3;
// float wheel4;

int main(int argc, char **argv)
{
    // Create ros node 'input_example' and advertise on ropic 'robotcommands'
    ros::init(argc, argv, "input_example");
    ros::NodeHandle n;
    ros::Publisher chatter_pub = n.advertise<std_msgs::Float64MultiArray>("gazebo_listener/motorsignals", 1000);
    // ros::Rate loop_rate(60);
    
    // Keyboard inputs
    double wheel1;
    double wheel2;
    double wheel3;
    double wheel4;

    std::cout << "Enter an x speed: ";
    std::cin >> x_vel;
    std::cout << "Enter a y speed: ";
    std::cin >> y_vel;
    std::cout << "Enter a rotational speed: ";
    std::cin >> w_vel;
    // std::cout << "Enter a speed for wheel 4: ";
    // std::cin >> wheel4;

    float robot_radius = 0.09;
    float wheel_radius = 0.03;
    wheel1 = x_vel/(2*3.1415*wheel_radius);
    wheel2 = y_vel/(2*3.1415*wheel_radius);
    wheel3 = x_vel/(2*3.1415*wheel_radius);
    wheel4 = y_vel/(2*3.1415*wheel_radius);

    active = true;
    dribbler = false;
    kick_vel = 0.0;
    chip_vel = 0.0;

    // Initialize RobotCommand message;
    // roboteam_msgs::RobotCommand command;
    std::vector<double> inputs = {wheel1, wheel2, wheel3, wheel4};
    std_msgs::Float64MultiArray command;

    command.layout.dim.push_back(std_msgs::MultiArrayDimension());
    command.layout.dim[0].size = 4;
    command.layout.dim[0].stride = 1;
    command.layout.dim[0].label = "speeds";
    // command.data = inputs;
    command.data.clear();
    command.data.insert(command.data.end(), inputs.begin(), inputs.end());
    // Fill the RobotCommand message with the keyboard inputs
/*
    command.id = id;
    command.active = active;
    command.x_vel = x_vel;
    command.y_vel = y_vel;
    command.w_vel = w_vel;
    command.dribbler = dribbler;
    command.kicker_vel = kick_vel;
    command.chipper_vel = chip_vel;
*/
    // Publish the command
    chatter_pub.publish(command);

    ROS_INFO_STREAM("command sent for robot: " << id);

    // loop_rate.sleep();
    ros::spin();

	return 0;
}
