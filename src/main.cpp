#include <QtNetwork>
#include <boost/asio.hpp>
#include <cmath>
#include <fstream>
#include <iostream>
#include <math.h>
#include <ros/ros.h>
#include <string>
#include <vector>

#include "roboteam_msgs/RobotCommand.h"
#include "roboteam_utils/Vector2.h"
#include "roboteam_utils/Vector2.h"
#include "roboteam_utils/constants.h"
#include "roboteam_utils/constants.h"
#include "roboteam_utils/grSim_Commands.pb.h"
#include "roboteam_utils/grSim_Commands.pb.h"
#include "roboteam_utils/grSim_Packet.pb.h"
#include "roboteam_utils/grSim_Packet.pb.h"
#include "roboteam_utils/normalize.h"
#include "roboteam_utils/normalize.h"
#include "std_msgs/Float64MultiArray.h"
#include "std_msgs/Bool.h"

#include "roboteam_robothub/packing.h"

namespace {

bool halt = false;

void processHalt(const std_msgs::Bool::ConstPtr &msg) {
    std::cout << "[RobotHub] Received a halt: " << (int) msg->data << "\n";
    halt = msg->data;
}

ros::Publisher pub;

QUdpSocket udpsocket;

void sendGRsimCommands(const roboteam_msgs::RobotCommand::ConstPtr &_msg) {
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
        roboteam_utils::Vector2 vel = roboteam_utils::Vector2(_msg->chipper_vel, 0);
        vel = vel.rotate(M_PI/4); // 45 degrees up.

        command->set_kickspeedx(vel.x);
    	command->set_kickspeedz(vel.y);
    }
    else {
    	command->set_kickspeedz(0);
    }

    command->set_spinner(_msg->dribbler);

	QByteArray dgram;
    dgram.resize(packet.ByteSize());
    packet.SerializeToArray(dgram.data(), dgram.size());

    // Send to IP address and port specified in grSim
    std::string grsim_ip = "127.0.0.1";
    int grsim_port = 20011;
    ros::param::get("grsim/ip", grsim_ip);
    ros::param::get("grsim/port", grsim_port);

    udpsocket.writeDatagram(dgram, QHostAddress(QString::fromStdString(grsim_ip)), grsim_port);
}

void sendGazeboCommands(const roboteam_msgs::RobotCommand::ConstPtr &_msg) {
    // ROS_INFO("received message for Gazebo!");

    float x_vel = _msg->x_vel;
    float y_vel = _msg->y_vel;
    float w = _msg->w;
    float robot_radius = 0.09;
    float wheel_radius = 0.03;
    float theta1 = 0.25 * M_PI;
    float theta2 = 0.75 * M_PI;
    float theta3 = 1.25 * M_PI;
    float theta4 = 1.75 * M_PI;
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

// IMPORTANT NOTE (I have to place it somewhere)
// Make sure ModemManager has been eradicated from
// your Ubuntu installation. otherwise, upon serial
// connection, it will dump about 30 characters of random bytes
// into the connection, screwing up the connection.
// ModemManager is a program/service of some sort.
// http://askubuntu.com/questions/216114/how-can-i-remove-modem-manager-from-boot
// sudo apt-get purge modemmanager + reboot or something should do the trick

namespace {

// http://www.boost.org/doc/libs/1_40_0/doc/html/boost_asio/overview/serial_ports.html

std::string const SERIAL_FILE_PATH = "/dev/ttyACM0";
bool serialPortOpen = false;
boost::asio::io_service io;
boost::asio::serial_port serialPort(io);

} // anonymous namespace

void sendSerialCommands(const roboteam_msgs::RobotCommand::ConstPtr &_msg) {
    // ROS_INFO("Send serial command");
    std::cout << "[RobotHub] Sending to robot " << _msg->id;

    // int id;
    // int robot_vel;
    // int ang;
    // bool rot_cclockwise;
    // int w;
    // uint8_t kick_force;
    // bool do_kick;
    // bool chip;
    // bool forced;
    // bool dribble_cclockwise;
    // uint8_t dribble_vel;

    // id = 7;
    // robot_vel = 2000;
    // ang = 300;
    // rot_cclockwise = true;
    // w = 1000;
    // kick_force = 200;
    // do_kick = true;
    // chip = false;
    // forced = true;
    // dribble_cclockwise = true;
    // dribble_vel = 5;

    // auto possibleMsg = rtt::createRobotPacket(
    //         id,
    //         robot_vel,
    //         ang,
    //         rot_cclockwise,
    //         w,
    //         kick_force,
    //         do_kick,
    //         chip,
    //         forced,
    //         dribble_cclockwise,
    //         dribble_vel
    //         );

    // if (!possibleMsg) {
    //     std::cout << "An error occurred while creating the message. Please look at the constraints of createRobotPacket(). Aborting.";
    //     // return 1;
    //     return;
    // }




    if (!serialPortOpen) {
        // Open serial port
        boost::system::error_code errorCode;
        serialPort.open(SERIAL_FILE_PATH, errorCode);
        switch (errorCode.value()) {
            case boost::system::errc::success:
                serialPortOpen = true;
                break;
            default:
                std::cerr << " ERROR! Could not open serial port!\n";
                return;
        }
    } 


    // Create message
    if (auto bytesOpt = rtt::createRobotPacket(*_msg)) {

    // if ((bool) possibleMsg) {

        // Success!
        auto bytes = *bytesOpt;
        // Write message to it
        serialPort.write_some(boost::asio::buffer(bytes.data(), bytes.size()));
        // TODO: @Hack Crutches! Packet length should be 7!
        serialPort.write_some(boost::asio::buffer(bytes.data(), 1));
        
        // Print the packet as binary - should not be needed
        // for (const auto& byte : bytes) {
            // std::cout << "\t" << rtt::byteToBinary(byte) << "\t" << std::to_string(byte) << "\n";
        // }
        
        // Listen for ack
        // CAREFUL! The first ascii character is the robot ID
        // _________________________________________
        // _____________                ____________
        // _____________ IN HEXADECIMAL ____________
        // _________________________________________

        uint8_t ackCode[3];
        // TODO: @Incomplete ack format is undefined/unstable! Guessing it is 2 bytes!
        serialPort.read_some(boost::asio::buffer(ackCode, 2));

        // If the second character in the response is an ascii 0 character,
        // it means sending the packet failed.
        if (ackCode[1] == '0') {
            std::cout << ", failed \n";
        } else if (ackCode[1] == '1') {
            std::cout << ", succeeded \n";
        } else {
            std::cout << " strange result: "
                      << (int) ackCode[1] 
                      << "\n";
        }

        // TODO: @Performance this should probably done in such a way that it doesn't
        // block ros::spin()
        // Altough it is pretty fast now at 1 packet per ms.
        // TODO: @Incomplete we do not handle the ACK here. Should probably influence the order or something (round robin?)
        
        // We're done

    } else {
        // Oh shit.
        std::cout << " Could not turn command into packet!\n";
    }
}

enum Mode {
    SERIAL,
    GRSIM,
    GAZEBO
} ;

Mode getMode() {
    std::string robot_output_target = "grsim";
    ros::param::getCached("robot_output_target", robot_output_target);
    if (robot_output_target == "grsim") {
        return Mode::GRSIM;
    } else if (robot_output_target == "serial") {
        return Mode::SERIAL;
    } else {
        return Mode::GAZEBO;
    }
}

void processRobotCommand(const roboteam_msgs::RobotCommand::ConstPtr &msg) {
    roboteam_msgs::RobotCommand command;

    if (halt) return;

    // TODO: @Safety I would like to use getcached here, but I would also like to
    // have the safety of utils' constants.

    bool normalizeField =  false;
    ros::param::getCached("normalize_field", normalizeField);

    if (normalizeField) {
        std::string ourSide = "left";
        ros::param::getCached("our_side", ourSide);

        if (ourSide == "right") {
            command = rtt::rotateRobotCommand(*msg);
        } else {
            command = *msg;
        }
    } else {
        command = *msg;
    }

    auto mode = getMode();
    if (mode == Mode::GRSIM) {
        sendGRsimCommands(msg);
    } else if (mode == Mode::GAZEBO) {
        sendGazeboCommands(msg);
    } else if (mode == Mode::SERIAL) {
        sendSerialCommands(msg);
    } else { // Default to grsim
        sendGRsimCommands(msg);
    }
}

} // anonymous namespace

int main(int argc, char *argv[]) {
    // Create ROS node called robothub and subscribe to topic 'robotcommands'
    ros::init(argc, argv, "robothub");
    ros::NodeHandle n;

    ros::Rate loop_rate(20);
    ros::Subscriber subRobotCommands = n.subscribe("robotcommands", 1/*0*/, processRobotCommand);
    pub = n.advertise<std_msgs::Float64MultiArray>("gazebo_listener/motorsignals", 1000);

    ros::Subscriber subHalt = n.subscribe("halt", 1, processHalt);
    
    while (ros::ok()) {
        loop_rate.sleep();
        ros::spinOnce();

        // Stop all robots if needed!
        if (halt) {
            roboteam_msgs::RobotCommand::Ptr command(new roboteam_msgs::RobotCommand());
            auto mode = getMode();
            for (int i = 0; i < 16; ++i) {
                command->id = i;
                if (mode == Mode::GRSIM) {
                    sendGRsimCommands(command);
                } else if (mode == Mode::GAZEBO) {
                    sendGazeboCommands(command);
                } else if (mode == Mode::SERIAL) {
                    sendSerialCommands(command);
                } else { // Default to grsim
                    sendGRsimCommands(command);
                }
            }
        }
    }

    return 0;
}
