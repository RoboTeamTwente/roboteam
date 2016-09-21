#include <QtNetwork>
#include <ros/ros.h>

#include "grSim_Packet.pb.h"
#include "grSim_Commands.pb.h"

#include "roboteam_msgs/RobotCommand.h"

#include <iostream>

void sendCommands(const roboteam_msgs::RobotCommand::ConstPtr &_msg)
{
    ROS_INFO_STREAM("message received for robot: " << _msg->id);

    grSim_Packet packet;
    bool yellow = true;
    packet.mutable_commands()->set_isteamyellow(yellow);
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
    command->set_velangular(_msg->w_vel);

    command->set_kickspeedx(_msg->kick_vel);
    command->set_kickspeedz(_msg->chip_vel);
    command->set_spinner(_msg->dribbler);

    QByteArray dgram;
    dgram.resize(packet.ByteSize());
    packet.SerializeToArray(dgram.data(), dgram.size());
    
    QUdpSocket udpsocket;
    QHostAddress _addr;
    quint16 _port;

    // Send to IP address and port specified in grSim
    _addr = "127.0.0.1";
    _port = 20011;
    udpsocket.writeDatagram(dgram, _addr, _port);
}

int main(int argc, char *argv[])
{
    // Create ROS node called robothub and subscribe to topic 'robotcommands'
    ros::init(argc, argv, "robothub");
    ros::NodeHandle n;
    ros::Rate loop_rate(60);
    ros::Subscriber sub = n.subscribe("robotcommands", 1000, sendCommands);
    loop_rate.sleep();
    ros::spin();
    
    return 0;
}