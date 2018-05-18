#include <boost/asio.hpp>
#include <boost/format.hpp>
#include <boost/optional.hpp>
namespace bf = boost;
#include <chrono>
#include <cstdint>
#include <fstream>
#include <iostream>
#include <string>
#include <thread>
#include <vector>
#include <math.h>

#include <ros/ros.h>

#include "roboteam_msgs/RobotCommand.h"
#include "roboteam_robothub/packing.h"


namespace rtt {

    void printcmd(roboteam_msgs::RobotCommand cmd){

        std::cout << "    id             : " << cmd.id                  << std::endl;
        std::cout << "    active         : " << (int)cmd.active         << std::endl;
        std::cout << "    x_vel          : " << cmd.x_vel               << std::endl;
        std::cout << "    y_vel          : " << cmd.y_vel               << std::endl;
        std::cout << "    w              : " << cmd.w                   << std::endl;
        std::cout << "    dribbler       : " << (int)cmd.dribbler       << std::endl;
        std::cout << "    kicker         : " << (int)cmd.kicker         << std::endl;
        std::cout << "    kicker_forced  : " << (int)cmd.kicker_forced  << std::endl;
        std::cout << "    kicker_vel     : " << cmd.kicker_vel          << std::endl;
        std::cout << "    chipper        : " << (int)cmd.chipper        << std::endl;
        std::cout << "    chipper_forced : " << (int)cmd.chipper_forced << std::endl;
        std::cout << "    chipper_vel    : " << cmd.chipper_vel         << std::endl;
        std::cout << "    geneva_state   : " << cmd.geneva_state        << std::endl;
    }

    void printbits(packed_protocol_message byteArr){
        for(int b = 0; b < 12; b++){

            std::cout << "    ";
            uint8_t byte = byteArr[b];
//            for (int i = 0; i < 8; i++) {
            for (int i = 7; i >= 0; i--) {
                if ((byte & (1 << i)) == (1 << i)) {
                    std::cout << "1";
                } else {
                    std::cout << "0";
                }
            }
            std::cout << std::endl;

        }
    }

    roboteam_msgs::RobotCommand createRobotCommand(int id, bool active, float x_vel, float y_vel, float w, bool dribbler, bool kicker, bool kicker_forced, float kicker_vel, bool chipper, bool chipper_forced, float chipper_vel, int geneva_state){
        roboteam_msgs::RobotCommand cmd;
        cmd.id              = id              ;
        cmd.active          = active          ;
        cmd.x_vel           = x_vel           ;
        cmd.y_vel           = y_vel           ;
        cmd.w               = w               ;
        cmd.dribbler        = dribbler        ;
        cmd.kicker          = kicker          ;
        cmd.kicker_forced   = kicker_forced   ;
        cmd.kicker_vel      = kicker_vel      ;
        cmd.chipper         = chipper         ;
        cmd.chipper_forced  = chipper_forced  ;
        cmd.chipper_vel     = chipper_vel     ;
        cmd.geneva_state    = geneva_state    ;
        return cmd;
    }

    void test(){
        ROS_INFO_STREAM("Running test()");

        roboteam_msgs::RobotCommand cmdMax;

//        cmdMax.x_vel = 5.6568542;   // Translates to Rho = 2047
//        cmdMax.y_vel = 5.6568542;   // Translates to Rho = 2047
        cmdMax.y_vel = -8.191;

        cmdMax.w = 16 * M_PI;       // Translates to velocity_angular = 511
        cmdMax.kicker_vel = 100;    // Translates to kick_chip_power = 255
        cmdMax.geneva_state = 4;

        LowLevelRobotCommand llrc = createLowLevelRobotCommand(cmdMax);

        std::cout << "\nRobotCommand Max" << std::endl;
        printcmd(cmdMax);
        std::cout << "\nLowLevelRobotCommand Max" << std::endl;
        rtt::printLowLevelRobotCommand(llrc);

        roboteam_msgs::RobotCommand cmdMin;

        llrc = createLowLevelRobotCommand(cmdMin);

        std::cout << "\nRobotCommand Min" << std::endl;
        printcmd(cmdMin);
        std::cout << "\nLowLevelRobotCommand Min" << std::endl;
        rtt::printLowLevelRobotCommand(llrc);

        boost::optional<packed_protocol_message> byteArr = createRobotPacket(llrc);

        if(byteArr) {
            std::cout << "\nRobotPacket Min" << std::endl;
            printbits(*byteArr);
        }
        else
            std::cout << "byteArr = null" << std::endl;

    }

    void test2(){

        union {
            float v;
            unsigned char b[4];
        } thing1, thing2, thing3;

        thing1.v = 1234;
        thing2.v = 5678;
        thing3.v = 9012;

        packed_robot_feedback x = {
            0b00011111,
            0b11110000,
            0b00000001,
            0b10000000,
            0b00001100,
            0b00000001,
            0b10000000,
            0b00110000,
            0b00000110,
            0b00000000,
            0b11000001,

            thing1.b[0],
            thing1.b[1],
            thing1.b[2],
            thing1.b[3],

            thing2.b[0],
            thing2.b[1],
            thing2.b[2],
            thing2.b[3],

            thing3.b[0],
            thing3.b[1],
            thing3.b[2],
            thing3.b[3],
        };

        LowLevelRobotFeedback llrf = createRobotFeedback(x);
        rtt::printLowLevelRobotFeedback(llrf);

    }

} // rtt

void test_cllrc_fakeWorld(){
	std::cout << "[test_cllrc_fakeWorld]" << std::endl;

	// Create a fake world
	roboteam_msgs::World world;
	std::cout << "[test_cllrc_fakeWorld] World created" << std::endl;

	// Create a fake robot
	roboteam_msgs::WorldRobot robot;
	// Set positions of robot 0
	robot.pos.x = 1;
	robot.pos.y = 1;
	robot.angle = M_PI / 2;
	std::cout << "[test_cllrc_fakeWorld] Robot created" << std::endl;

	world.us.push_back(robot);
	std::cout << "[test_cllrc_fakeWorld] Robot added to world" << std::endl;

	// Create a fake command
	roboteam_msgs::RobotCommand cmd;
	cmd.id = 0;

	rtt::LowLevelRobotCommand llrc = rtt::createLowLevelRobotCommand(cmd, world);
	rtt::printLowLevelRobotCommand(llrc);
}

int main(int argc, char *argv[]) {

	test_cllrc_fakeWorld();	return 0;

    std::cout << "argc: " << argc << std::endl;
    for(int i = 1; i < argc; i++){
        std::cout << argv[i] << " ";
    }
    std::cout << std::endl;

    if(argc == 14){
        std::cout << "Enough arguments to make a RobotCommand. Attempting.." << std::endl;
        roboteam_msgs::RobotCommand cmd = rtt::createRobotCommand(
            std::stoi(argv[1]),         //id
            strcmp(argv[2], "1") == 0,  //active
            std::stof(argv[3]),         //x_vel
            std::stof(argv[4]),         //y_vel
            std::stof(argv[5]),         //w
            strcmp(argv[6], "1") == 0,  //dribbler
            strcmp(argv[7], "1") == 0,  //kicker
            strcmp(argv[8], "1") == 0,  //kicker_forced
            std::stof(argv[9]),         //kicker_vel
            strcmp(argv[10], "1") == 0, //chipper
            strcmp(argv[11], "1") == 0, //chipper_forced
            std::stof(argv[12]),        //chipper_vel
            std::stoi(argv[13])         //geneva_state
        );

        std::cout << "###" << std::endl;
        rtt::printcmd(cmd);

        std::cout << "###" << std::endl;
        rtt::LowLevelRobotCommand llrc = rtt::createLowLevelRobotCommand(cmd);
        rtt:printLowLevelRobotCommand(llrc);

        std::cout << "###" << std::endl;
        boost::optional<rtt::packed_protocol_message> byteArr = rtt::createRobotPacket(llrc);
        if(byteArr) rtt::printbits(*byteArr);

        std::cout << "###" << std::endl;
    } else {
        rtt::test2();
    }


    return 0;

    rtt::test();



    return 0;
}
