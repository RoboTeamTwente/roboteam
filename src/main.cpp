#include "grSim_Packet.pb.h"
#include "grSim_Commands.pb.h"
#include "roboteam_msgs/RobotCommand.h"
#include "std_msgs/Float64MultiArray.h"

#include <iostream>
#include <string>
#include <QtNetwork>
#include <ros/ros.h>

#include <vector>
#include <math.h>

#define PI 3.14159265

ros::Publisher pub;

void sendGRsimCommands(const roboteam_msgs::RobotCommand::ConstPtr &_msg)
{
    // ROS_INFO_STREAM("received message for GRsim");
    grSim_Packet packet;

    std::string color;
    ros::param::get("our_color", color);

    packet.mutable_commands()->set_isteamyellow(color == "yellow");
    packet.mutable_commands()->set_timestamp(0.0);

    // Initialize a grSim command (grSim is the robocup SSL simulator created by Parsians)
    grSim_Robot_Command* command = packet.mutable_commands()->add_robot_commands();
    command->set_id(_msg->id);

    // Fill the grSim command with the received values. Set either wheelspeed or robotspeed
    command->set_wheelsspeed(false);
    // command->set_wheel1(0.0);
    // command->set_wheel2(0.0);
    // command->set_wheel3(0.0);
    // command->set_wheel4(0.0);
    command->set_veltangent(_msg->x_vel);
    command->set_velnormal(_msg->y_vel);
    command->set_velangular(_msg->w);

	if(_msg->kicker){
    	command->set_kickspeedx(_msg->kicker_vel);
    }
    else {
    	command->set_kickspeedx(0);
    }
    if(_msg->chipper){

    	command->set_kickspeedz(_msg->chipper_vel);
    }
    else {
    	command->set_kickspeedz(0);
    }

    command->set_spinner(_msg->dribbler);

	QByteArray dgram;
    dgram.resize(packet.ByteSize());
    packet.SerializeToArray(dgram.data(), dgram.size());

    QUdpSocket udpsocket;

    // Send to IP address and port specified in grSim
    std::string grsim_ip = "127.0.0.1";
    int grsim_port = 20011;
    ros::param::get("grsim/ip", grsim_ip);
    ros::param::get("grsim/port", grsim_port);
    udpsocket.writeDatagram(dgram, QHostAddress(QString::fromStdString(grsim_ip)), grsim_port);
}

void sendGazeboCommands(const roboteam_msgs::RobotCommand::ConstPtr &_msg)
{
    // ROS_INFO("received message for Gazebo!");

    float x_vel = _msg->x_vel;
    float y_vel = _msg->y_vel;
    float w = _msg->w;
    float robot_radius = 0.09;
    float wheel_radius = 0.03;
    float theta1 = 0.25*PI;
    float theta2 = 0.75*PI;
    float theta3 = 1.25*PI;
    float theta4 = 1.75*PI;
    float fr_wheel = (-1/sin(theta1)*x_vel + 1/cos(theta1)*y_vel + robot_radius*w) / wheel_radius;
    float fl_wheel = (-1/sin(theta2)*x_vel + 1/cos(theta2)*y_vel + robot_radius*w) / wheel_radius;
    float bl_wheel = (-1/sin(theta3)*x_vel + 1/cos(theta3)*y_vel + robot_radius*w) / wheel_radius;
    float br_wheel = (-1/sin(theta4)*x_vel + 1/cos(theta4)*y_vel + robot_radius*w) / wheel_radius;

    std::vector<double> inputs = {-fr_wheel, -fl_wheel, -bl_wheel, -br_wheel};
    std_msgs::Float64MultiArray command;

    command.layout.dim.push_back(std_msgs::MultiArrayDimension());
    command.layout.dim[0].size = 4;
    command.layout.dim[0].stride = 1;
    command.layout.dim[0].label = "speeds";

    command.data.clear();
    command.data.insert(command.data.end(), inputs.begin(), inputs.end());

    pub.publish(command);
}

void processRobotCommand(const roboteam_msgs::RobotCommand::ConstPtr &msg) {
    std::string robot_output_target = "grsim";
    ros::param::get("robot_output_target", robot_output_target);
    if (robot_output_target == "grsim") {
        sendGRsimCommands(msg);
    } else if (robot_output_target == "gazebo") {
        sendGazeboCommands(msg);
    } else if (robot_output_target == "serial") {
        // sendSerialCommands(msg);
    } else { // Default to grsim
        sendGRsimCommands(msg);
    }
}

int main(int argc, char *argv[]) {
    // Create ROS node called robothub and subscribe to topic 'robotcommands'
    ros::init(argc, argv, "robothub");
    ros::NodeHandle n;
    ros::Rate loop_rate(60);
    ros::Subscriber subRobotCommands = n.subscribe("robotcommands", 1, processRobotCommand);
    pub = n.advertise<std_msgs::Float64MultiArray>("gazebo_listener/motorsignals", 1000);
    loop_rate.sleep();
    ros::spin();

    return 0;
}
