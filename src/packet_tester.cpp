#include <boost/asio.hpp>
#include <boost/format.hpp>
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

        std::cout << "\nRobotCommand" << std::endl;
        printcmd(cmdMax);
        std::cout << "\nLowLevelRobotCommand" << std::endl;
        printllrc(llrc);

    }


} // rtt

int main(int argc, char *argv[]) {

    rtt::test();

    return 0;
}
