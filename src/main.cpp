#include <QtNetwork>
#include <boost/asio.hpp>
#include <cmath>
#include <fstream>
#include <iostream>
#include <math.h>
#include <ros/ros.h>
#include <string>
#include <vector>
#include <chrono>
#include <boost/optional.hpp>
namespace b = boost;

#include "roboteam_msgs/RobotCommand.h"
#include "roboteam_msgs/World.h"
#include "roboteam_utils/LastWorld.h"
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
#include "std_msgs/Bool.h"
#include "std_msgs/Float64MultiArray.h"

#include "roboteam_robothub/packing.h"

namespace {

bool halt = false;

void processHalt(const std_msgs::Bool::ConstPtr &msg) {
    std::cout << "[RobotHub] Received a halt: " << (int) msg->data << "\n";
    halt = msg->data;
}

ros::Publisher pub;

QUdpSocket udpsocket;

void sendGRsimCommands(const roboteam_msgs::RobotCommand & _msg) {
    // ROS_INFO_STREAM("received message for GRsim");
    grSim_Packet packet;

    std::string color;
    ros::param::get("our_color", color);

    packet.mutable_commands()->set_isteamyellow(color == "yellow");
    packet.mutable_commands()->set_timestamp(0.0);

    // Initialize a grSim command (grSim is the robocup SSL simulator created by Parsians)
    grSim_Robot_Command* command = packet.mutable_commands()->add_robot_commands();
    command->set_id(_msg.id);

    // Fill the grSim command with the received values. Set either wheelspeed or robotspeed
    command->set_wheelsspeed(false);
    // command->set_wheel1(0.0);
    // command->set_wheel2(0.0);
    // command->set_wheel3(0.0);
    // command->set_wheel4(0.0);
    command->set_veltangent(_msg.x_vel);
    command->set_velnormal(_msg.y_vel);
    command->set_velangular(_msg.w);

	if(_msg.kicker){
    	command->set_kickspeedx(_msg.kicker_vel);
    }
    else {
    	command->set_kickspeedx(0);
    }
    if(_msg.chipper){
        rtt::Vector2 vel = rtt::Vector2(_msg.chipper_vel, 0);
        vel = vel.rotate(M_PI/4); // 45 degrees up.

        command->set_kickspeedx(vel.x);
    	command->set_kickspeedz(vel.y);
    }
    else {
    	command->set_kickspeedz(0);
    }

    command->set_spinner(_msg.dribbler);

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

void sendGazeboCommands(const roboteam_msgs::RobotCommand & _msg) {
    // ROS_INFO("received message for Gazebo!");

    float x_vel = _msg.x_vel;
    float y_vel = _msg.y_vel;
    float w = _msg.w;
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

std::string SERIAL_FILE_PATH = "/dev/ttyACM3";
bool serialPortOpen = false;
boost::asio::io_service io;
boost::asio::serial_port serialPort(io);

int acks = 0;
int nacks = 0;

} // anonymous namespace

void sendSerialCommands(const roboteam_msgs::RobotCommand &_msg) {

    if (!serialPortOpen) {
        // Open serial port
        std::cout << "Opening serial port...\n";
        boost::system::error_code errorCode;
        serialPort.open(SERIAL_FILE_PATH, errorCode);
        switch (errorCode.value()) {
            case boost::system::errc::success:
                serialPortOpen = true;
                std::cout << "Port opened.\n";
                break;
            default:
                std::cerr << " ERROR! Could not open serial port!\n";
                return;
        }
    } 

    // Create message
    
    b::optional<roboteam_msgs::World> worldOpt;
    if (rtt::LastWorld::have_received_first_world()) {
        worldOpt = rtt::LastWorld::get();
    }

    if (auto bytesOpt = rtt::createRobotPacket(_msg, worldOpt)) {
        // Success!

        auto bytes = *bytesOpt;

        // Uncomment for debug info
        // std::cout << "Byte size: " << bytes.size() << "\n";
        // std::cout << "Bytes: \n";
        // for (uint8_t b : bytes) {
            // printf("%s (%x)\n", rtt::byteToBinary(b).c_str(), b);

        // }

        // Write message to it
        b::asio::write(serialPort, boost::asio::buffer(bytes.data(), bytes.size()));

        // Listen for ack
        // CAREFUL! The first ascii character is the robot ID
        // _________________________________________
        // _____________                ____________
        // _____________ IN HEXADECIMAL ____________
        // _________________________________________

        // int const returnSize = (2*12)+2;
        int const returnSize = 2;
        std::array<uint8_t, returnSize> ackCode;
        ackCode.fill('!');
        // serialPort.read_some(boost::asio::buffer(ackCode.data(), returnSize));
        b::asio::read(serialPort, boost::asio::buffer(ackCode.data(), returnSize));

        // Uncomment for debug info
        // std::cout << "AckCode size: " << ackCode.size() << "\n";
        // std::cout << "ackCodes: \n";
        // 
        // for (int i = 0; i < returnSize; i += 2) {
            // // uint8_t b = ackCode[i];
            // printf("%c%c\n", ackCode[i], ackCode[i + 1]);
        // }

        auto ack = rtt::decodeOldACK(ackCode);

        if (ack.robotACK) {
            acks++;
        } else {
            nacks++;
        }

        int const MAX_NACKS = 20;
        if (nacks > MAX_NACKS) {
            // std::cout << "Got " << MAX_NACKS << " or more nacks over the past second of sending packets.\n";
        }

        // TODO: @Performance this should probably done in such a way that it doesn't
        // block ros::spin()
        // Altough it is pretty fast now at 1 packet per ms.
        // TODO: @Incomplete we do not handle the ACK here. Should probably influence the order or something (round robin?)
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
    roboteam_msgs::RobotCommand command = *msg;

    if (halt) return;

    auto mode = getMode();
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

bool hasArg(std::vector<std::string> const & args, std::string const & arg) {
    return std::find(args.begin(), args.end(), arg) != args.end();
}

void mergeAndProcessRobotCommand(const ros::MessageEvent<roboteam_msgs::RobotCommand const>& msgEvent) {

    auto const & publisherName = msgEvent.getPublisherName();
    bool isYellow = publisherName.find("yellow") != std::string::npos;
    std::string prefix;

    auto msg = *msgEvent.getConstMessage();

    if (isYellow) {
        prefix = "/yellow";
    } else {
        msg.id += 3;
        prefix = "/blue";
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

bool MERGING = false;

} // anonymous namespace

int main(int argc, char *argv[]) {
    std::vector<std::string> args(argv, argv + argc);

    if (hasArg(args, "--help") || hasArg(args, "-h")) {
        std::cout << "--help or -h for help. --merge-mode to merge yellow and blue into robots 0-2 and 3-5 of the yellow team respectively.\n";
        return 0;
    }

    MERGING = hasArg(args, "--merge-mode");

    // Create ROS node called robothub and subscribe to topic 'robotcommands'
    ros::init(argc, argv, "robothub");
    ros::NodeHandle n;

    // TODO: Do this with a proper flag
    // if (argc > 1) {
        // SERIAL_FILE_PATH = std::string(argv[1]);
    // } else if (ros::param::has("serialOut")) {
        // ros::param::get("serialOut", SERIAL_FILE_PATH);
    // }

    auto mode = getMode();
    if (mode == Mode::SERIAL) {
        ROS_INFO("[RobotHub] Serial Device: %s\n", SERIAL_FILE_PATH.c_str());
    }

    ros::Subscriber mergingRobotCommandsBlue;
    ros::Subscriber mergingRobotCommandsYellow;

    if (MERGING) {
        mergingRobotCommandsBlue = n.subscribe("/blue/robotcommands", 1000, mergeAndProcessRobotCommand);
        mergingRobotCommandsYellow = n.subscribe("/yellow/robotcommands", 1000, mergeAndProcessRobotCommand);
    }

    int roleHz = 120;
    ros::param::get("role_iterations_per_second", roleHz);
    ros::Rate loop_rate(roleHz);

    std::cout << "[RobotHub] Refresh rate: " << roleHz << "hz\n";

    ros::Subscriber subRobotCommands = n.subscribe("robotcommands", 1000, processRobotCommand);
    pub = n.advertise<std_msgs::Float64MultiArray>("gazebo_listener/motorsignals", 1000);

    ros::Subscriber subHalt = n.subscribe("halt", 1, processHalt);

    // Creates the callbacks for world and geom
    rtt::WorldAndGeomCallbackCreator wgcc;

    using namespace std;
    using namespace std::chrono; 
    
    high_resolution_clock::time_point lastStatistics = high_resolution_clock::now();
    
    while (ros::ok()) {
        // std::cout << "----- DOING A CYCLE! -----\n";

        loop_rate.sleep();
        ros::spinOnce();

        mode = getMode();

        // Stop all robots if needed!
        if (halt) {

            roboteam_msgs::RobotCommand command;
            for (int i = 0; i < 16; ++i) {
                command.id = i;
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
        
        if (mode == Mode::SERIAL) {
            auto timeDiff = high_resolution_clock::now() - lastStatistics;

            if (duration_cast<milliseconds>(timeDiff).count() > 1000) {
                lastStatistics = high_resolution_clock::now();

                auto total = acks + nacks;
                auto ackPercent = static_cast<int>((acks / (double) total) * 100);
                auto nackPercent = static_cast<int>((nacks / (double) total) * 100);

                if (total == 0) {
                    ackPercent = 0;
                    nackPercent = 0;
                }

                std::cout << "-----------------------------------\n";
                std::cout << "Sent messages the past second: " << total << "\n";
                std::cout << "Capacity: " << std::floor(total / 360.0) << "%\n";
                std::cout << "Acks: " << acks << " (" << ackPercent << ")\n";
                std::cout << "Nacks: " << nacks << " (" << nackPercent << ")\n";
                acks=0;
                nacks=0;
            }
        }
    }

    return 0;
}
