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
#include "roboteam_msgs/RobotCommand.h"


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

    void printllrc(LowLevelRobotCommand llrc){
        std::cout << "    id                 : " << llrc.id << std::endl;
        std::cout << "    rho                : " << llrc.rho << std::endl;
        std::cout << "    theta              : " << llrc.theta << std::endl;
        std::cout << "    driving_reference  : " << llrc.driving_reference << std::endl;
        std::cout << "    use_cam_inf        : " << llrc.use_cam_info << std::endl;
        std::cout << "    velocity_angular   : " << llrc.velocity_angular << std::endl;
        std::cout << "    debug_info         : " << llrc.debug_info << std::endl;
        std::cout << "    do_kick            : " << llrc.do_kick << std::endl;
        std::cout << "    do_chip            : " << llrc.do_chip << std::endl;
        std::cout << "    kick_chip_forced   : " << llrc.kick_chip_forced << std::endl;
        std::cout << "    kick_chip_power    : " << llrc.kick_chip_power << std::endl;
        std::cout << "    velocity_dribbler  : " << llrc.velocity_dribbler << std::endl;
        std::cout << "    geneva_drive_state : " << llrc.geneva_drive_state << std::endl;
        std::cout << "    cam_position_x     : " << llrc.cam_position_x << std::endl;
        std::cout << "    cam_position_y     : " << llrc.cam_position_y << std::endl;
        std::cout << "    cam_rotation       : " << llrc.cam_rotation << std::endl;
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
        printllrc(llrc);

        roboteam_msgs::RobotCommand cmdMin;

        llrc = createLowLevelRobotCommand(cmdMin);

        std::cout << "\nRobotCommand Min" << std::endl;
        printcmd(cmdMin);
        std::cout << "\nLowLevelRobotCommand Min" << std::endl;
        printllrc(llrc);

        boost::optional<packed_protocol_message> byteArr = createRobotPacket(llrc);

        if(byteArr) {
            std::cout << "\nRobotPacket Min" << std::endl;
            printbits(*byteArr);
        }
        else
            std::cout << "byteArr = null" << std::endl;

    }


} // rtt

int main(int argc, char *argv[]) {

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
        rtt:printllrc(llrc);

        std::cout << "###" << std::endl;
        boost::optional<rtt::packed_protocol_message> byteArr = rtt::createRobotPacket(llrc);
        if(byteArr) rtt::printbits(*byteArr);

        std::cout << "###" << std::endl;
    }


    return 0;

    rtt::test();



    return 0;
}
