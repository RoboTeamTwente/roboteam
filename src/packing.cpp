#pragma clang diagnostic push
#pragma ide diagnostic ignored "hicpp-signed-bitwise"
#include "packing.h"

#include <cassert>
#include <cstdint>
#include <iostream>
#include <memory>
#include <math.h>
#include <ros/ros.h>

#include "roboteam_msgs/RobotCommand.h"
#include "roboteam_msgs/RobotFeedback.h"
#include "roboteam_msgs/World.h"
#include "roboteam_utils/Vector2.h"
#include "roboteam_utils/Math.h"
#include "utilities.h"

namespace rtt {
namespace robothub {
/**
 * Creates a low level robot command, i.e. a command from which you can construct a robot packet
 * only from bitshifts, and no other funky angle sin/cos velocity arithmetic. createRobotPacket
 * uses this internally to convert a RobotCommand into something workable.
 */
LowLevelRobotCommand createLowLevelRobotCommand(const roboteam_msgs::RobotCommand& command,
        const std::shared_ptr<roboteam_msgs::World>& worldOpt)
{

    double kick_chip_power = fmax(command.kicker_vel, command.chipper_vel);
    double rho = sqrt(command.x_vel*command.x_vel+command.y_vel*command.y_vel);
    double theta = atan2(command.y_vel, command.x_vel);

    LowLevelRobotCommand llrc{};
    // Units           Represented values
    llrc.id = command.id;                                                           // [0, 15]         [0, 15]
    llrc.rho = (int) floor(rho*250);                                                // [0, 2047]       [0, 8.191]
    llrc.theta = (int) floor(theta*1023.5/M_PI);                                    // [-1024, 1023]   [-pi, pi>

    llrc.driving_reference = false;                                                 // [0, 1]          {true, false}
    llrc.use_cam_info = false;                                                      // [0, 1]          {true, false}
    llrc.use_angle = command.use_angle;                                             // [0, 1]          {true, false}
    if (command.use_angle) {
        llrc.velocity_angular = (int) floor(command.w*(511/M_PI));                  // [-512, 511]     [-pi, pi]

    }
    else {
        llrc.velocity_angular = (int) floor(command.w*(511/(8*2*M_PI)));            // [-512, 511]     [-8*2pi, 8*2pi]
    }
    llrc.debug_info = true;                                                         // [0, 1]          {true, false}
    llrc.do_kick = command.kicker;                                                  // [0, 1]          {true, false}
    llrc.do_chip = command.chipper;                                                 // [0, 1]          {true, false}
    llrc.kick_chip_forced = command.kicker_forced || command.chipper_forced;        // [0, 1]          {true, false}
    llrc.kick_chip_power = (int) floor(kick_chip_power*255/8);                      // [0, 255]        [0, 100]%
    llrc.velocity_dribbler = command.dribbler;                                      // [0, 31]        [0, 100]%

    llrc.geneva_drive_state = command.geneva_state;                                 // [(0)1, 5]       [-2, 2]
    llrc.cam_position_x = 0;                                                        // [-4096, 4095]   [-10.24, 10.23]
    llrc.cam_position_y = 0;                                                        // [-4096, 4095]   [-10.24, 10.23]
    llrc.cam_rotation = 0;                                                          // [-1024, 1023]   [-pi, pi>

    if (worldOpt) {
        try {
            std::shared_ptr<roboteam_msgs::WorldRobot> findBot = utils::getWorldBot(command.id, true, *worldOpt);
            roboteam_msgs::WorldRobot robot;
            if (findBot) {
                robot = *findBot;
                llrc.cam_position_x = 0;//(int) (robot.pos.x/10.24*4096); // set to 0 to avoid mystery stop bug
                llrc.cam_position_y = 0;//(int) (robot.pos.y/10.24*4096); // set to 0 to avoid mystery stop bug
                llrc.cam_rotation = (int) floor(robot.angle/M_PI*1024);
                llrc.use_cam_info = true;
            }

        }
        catch (std::out_of_range e) {
            ROS_WARN_STREAM_THROTTLE(1, "[createLowLevelRobotCommand] Robot " << command.id
                                                                              << " not present in World! Not adding camera data");
        }
        catch (std::exception e) {
            ROS_WARN_STREAM_THROTTLE(1,
                    "[createLowLevelRobotCommand] Something went wrong while adding camera data for robot "
                            << command.id << " : " << e.what());
        }
    }

    return llrc;
}

bool inRange(int val, int min, int max)
{
    return (min<=val && val<=max);
}

/**
 * Checks if the values of the LowLevelRobotCommand are all within the correct range
 * @param llrc : The LowLevelRobotCommand to be checked
 * @returns true if the LowLevelRobotCommand is correct, false otherwise
 */
bool validateRobotPacket(LowLevelRobotCommand llrc)
{

    // inRange(val, min, max) is inclusive!;
    // inRange(-5, -5, 10) -> true; // inRange(14, -5, 10) -> false;
    // inRange(10, -5, 10) -> true; // inRange(-8, -5, 10) -> false;

    // Holds if all the values of the command are in range
    bool valuesInRange = true;

    valuesInRange &= inRange(llrc.id, 0, 15);
    valuesInRange &= inRange(llrc.rho, 0, 2047);
    valuesInRange &= inRange(llrc.theta, -1024, 1023);
    valuesInRange &= inRange(llrc.velocity_angular, -512, 511);
    valuesInRange &= inRange(llrc.kick_chip_power, 0, 255);
    valuesInRange &= inRange(llrc.velocity_dribbler, 0, 31);
    valuesInRange &= inRange(llrc.geneva_drive_state, 0, 5);
    valuesInRange &= inRange(llrc.cam_position_x, -4096, 4095);
    valuesInRange &= inRange(llrc.cam_position_y, -4096, 4095);
    valuesInRange &= inRange(llrc.cam_rotation, -1024, 1023);

    return valuesInRange;
}

/* As described in this comment https://roboteamtwente2.slack.com/files/U6CPQLJ6S/F9V330Z2N/packet_proposal.txt */
/**
 * Converts a LowLevelRobotCommand into a packet_protocol_message
 * @param llrc : LowLevelRobotCommand to be converted into a packet_protocol_message
 * @returns an optional packed_protocol_message, if the LowLevelRobotCommand can be validated
 */
std::shared_ptr<packed_protocol_message> createRobotPacket(LowLevelRobotCommand llrc)
{

    packed_protocol_message byteArr;


    byteArr[0] = static_cast<uint8_t>( // ID 1 byte
            0b11111111 & llrc.id
    );

    byteArr[1] = static_cast<uint8_t>( // All zeros
            0
    );

    byteArr[2] = static_cast<uint8_t>( // rho  first byte
            0b11111111 & llrc.rho >> 3
    );

    byteArr[3] = static_cast<uint8_t>(  // rho last 3 bits | theta first 5 bits
            (0b11100000 & (llrc.rho << 5))
            |
            (0b00011111 & (llrc.theta >> 6))
    );

    byteArr[4] = static_cast<uint8_t>( // theta last 6 bits | angle first 2 bits
            (0b11111100 & (llrc.theta << 2))
            |
            (0b00000011 & (llrc.velocity_angular >> 8))
    );

    byteArr[5] = static_cast<uint8_t>( // angle last byte
            0b11111111 & llrc.velocity_angular
    );

    byteArr[6] = static_cast<uint8_t>( // kick - chip power 1 byte
            0b11111111 & llrc.kick_chip_power
    );

    byteArr[7] = static_cast<uint8_t>( // K C F De UC 3 bits of geneva
            (0b10000000 & (llrc.do_kick << 7))
            |
            (0b01000000 & (llrc.do_chip << 6))
            |
            (0b00100000 & (llrc.kick_chip_forced << 5))
            |
            (0b00010000 & (llrc.debug_info << 4))
            |
            (0b00001000 & (llrc.use_cam_info << 3))
            |
            (0b00000111 & (llrc.geneva_drive_state))
    );

    byteArr[8] = static_cast<uint8_t>( // 5 bits dribble vel | first 3 bits of cam rotation
            (0b11111000 & (llrc.velocity_dribbler << 3))
            |
            (0b00000111 & (llrc.cam_rotation >> 8))
    );

    byteArr[9] = static_cast<uint8_t>( // Last byte of cam rotation
            0b11111111 & llrc.cam_rotation // TODO check
    );

    return std::make_shared<packed_protocol_message>(byteArr);
}

/**
 * Converts a LowLevelRobotFeedback into a roboteam_msgs::RobotFeedback
 * TODO this function might be redundant, because LowLevelRobotFeedback and roboteam_msgs::RobotFeedback have exactly the same values
 * TODO consider removing this function, and immediately converting packed_robot_feedback to roboteam_msgs::RobotFeedback
 * @param feedback : LowLevelRobotFeedback to be converted
 * @returns a roboteam_msgs::RobotFeedback object
 */
roboteam_msgs::RobotFeedback toRobotFeedback(LowLevelRobotFeedback feedback)
{
    roboteam_msgs::RobotFeedback msg;

    msg.id = feedback.id;
    msg.wheelLeftFront = feedback.wheelLeftFront;
    msg.wheelRightFront = feedback.wheelRightFront;
    msg.wheelLeftBack = feedback.wheelLeftBack;
    msg.wheelRightBack = feedback.wheelRightBack;

    msg.genevaDriveState = feedback.genevaDriveState;
    msg.batteryState = feedback.batteryState;

    msg.position_x = feedback.position_x;
    msg.position_y = feedback.position_y;

    msg.rho = feedback.rho;
    msg.theta = feedback.theta;
    msg.rotation = feedback.orientation;

    msg.angularVelocity = feedback.angularVelocity;
    msg.ballSensor = feedback.ballSensor;

    msg.acceleration_x = feedback.acceleration_x;
    msg.acceleration_y = feedback.acceleration_y;
    msg.velocity_angular = feedback.velocity_angular;

    return msg;
}

/**
 * Converts the bytes received from the robot into a LowLevelRobotFeedback object
 * @param bitsnbytes The bytes from the robot
 * @returns a LowLevelRobotFeedback struct
 */
LowLevelRobotFeedback createRobotFeedback(packed_robot_feedback bitsnbytes) {
    struct LowLevelRobotFeedback feedback;

    int offset = 0;

    // 0 aaaaabcd
    feedback.id = bitsnbytes[offset+0] >> 3;
    feedback.wheelLeftFront = (bitsnbytes[offset+0] & 4)==4;
    feedback.wheelRightFront = (bitsnbytes[offset+0] & 2)==2;
    feedback.wheelLeftBack = (bitsnbytes[offset+0] & 1)==1;

    // 1 efghhhhh
    feedback.wheelRightBack = (bitsnbytes[offset+1] & (1 << 7))==(1 << 7);
    feedback.genevaDriveState = (bitsnbytes[offset+1] & (1 << 6))==(1 << 6);
    feedback.batteryState = (bitsnbytes[offset+1] & (1 << 5))==(1 << 5);
    feedback.position_x = (bitsnbytes[offset+1] & 0b00011111) << 8;

    // 2 hhhhhhhh
    feedback.position_x |= bitsnbytes[offset+2];

    // 3 kkkkkkkk
    feedback.position_y = bitsnbytes[offset+3] << 5;

    // 4 kkkkkmmm
    feedback.position_y |= bitsnbytes[offset+4] >> 3;
    feedback.rho = (bitsnbytes[offset+4] & 0b00000111) << 8;

    // 5 mmmmmmmm
    feedback.rho |= bitsnbytes[offset+5];

    // 6 oooooooo
    feedback.theta = bitsnbytes[offset+6] << 3;

    // 7 oooppppp
    feedback.theta |= bitsnbytes[offset+7] >> 5;
    feedback.orientation = (bitsnbytes[offset+7] & 0b00011111) << 6;

    // 8 ppppppqq
    feedback.orientation |= bitsnbytes[offset+8] >> 2;
    feedback.angularVelocity = (bitsnbytes[offset+8] & 0b00000011) << 9;

    // 9 qqqqqqqq
    feedback.angularVelocity |= bitsnbytes[offset+9] << 1;

    // 10 qsssssss
    feedback.angularVelocity |= (bitsnbytes[offset+10] & 0b10000000) >> 7;
    feedback.ballSensor = bitsnbytes[offset+10] & 0b01111111;

    union {
        float v;
        unsigned char b[4];
    } f;

    f.b[3] = bitsnbytes[offset+11];
    f.b[2] = bitsnbytes[offset+12];
    f.b[1] = bitsnbytes[offset+13];
    f.b[0] = bitsnbytes[offset+14];
    feedback.acceleration_x = f.v;

    f.b[3] = bitsnbytes[offset+15];
    f.b[2] = bitsnbytes[offset+16];
    f.b[1] = bitsnbytes[offset+17];
    f.b[0] = bitsnbytes[offset+18];
    feedback.acceleration_y = f.v;

    f.b[3] = bitsnbytes[offset+19];
    f.b[2] = bitsnbytes[offset+20];
    f.b[1] = bitsnbytes[offset+21];
    f.b[0] = bitsnbytes[offset+22];
    feedback.velocity_angular = f.v;

    return feedback;

}

void printRobotCommand(const roboteam_msgs::RobotCommand& cmd)
{
    std::cout << "RobotCommand: " << std::endl;

    std::cout << "    id             : " << cmd.id << std::endl;
    std::cout << "    active         : " << (int) cmd.active << std::endl;
    std::cout << "    x_vel          : " << cmd.x_vel << std::endl;
    std::cout << "    y_vel          : " << cmd.y_vel << std::endl;
    std::cout << "    w              : " << cmd.w << std::endl;
    std::cout << "    use_angle      : " << (int) cmd.use_angle << std::endl;
    std::cout << "    dribbler       : " << (int) cmd.dribbler << std::endl;
    std::cout << "    kicker         : " << (int) cmd.kicker << std::endl;
    std::cout << "    kicker_forced  : " << (int) cmd.kicker_forced << std::endl;
    std::cout << "    kicker_vel     : " << cmd.kicker_vel << std::endl;
    std::cout << "    chipper        : " << (int) cmd.chipper << std::endl;
    std::cout << "    chipper_forced : " << (int) cmd.chipper_forced << std::endl;
    std::cout << "    chipper_vel    : " << cmd.chipper_vel << std::endl;
    std::cout << "    geneva_state   : " << cmd.geneva_state << std::endl;

    std::cout << std::endl;
}

void printLowLevelRobotCommand(const LowLevelRobotCommand& llrc)
{
    std::cout << "LowLevelRobotCommand: " << std::endl;

    std::cout << "    id                 : " << llrc.id << std::endl;
    std::cout << "    rho                : " << llrc.rho << std::endl;
    std::cout << "    theta              : " << llrc.theta << std::endl;
    std::cout << "    driving_reference  : " << llrc.driving_reference << std::endl;
    std::cout << "    use_cam_info       : " << llrc.use_cam_info << std::endl;
    std::cout << "    use_angle          : " << llrc.use_angle << std::endl;
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

    std::cout << std::endl;
}

void printLowLevelRobotFeedback(const LowLevelRobotFeedback& llrf)
{
    std::cout << "LowLevelRobotFeedback: " << std::endl;

    std::cout << "    id               : " << llrf.id << std::endl;
    std::cout << "    wheelLeftFront   : " << llrf.wheelLeftFront << std::endl;
    std::cout << "    wheelRightFront  : " << llrf.wheelRightFront << std::endl;
    std::cout << "    wheelLeftBack    : " << llrf.wheelLeftBack << std::endl;
    std::cout << "    wheelRightBack   : " << llrf.wheelRightBack << std::endl;
    std::cout << "    genevaDriveState : " << llrf.genevaDriveState << std::endl;
    std::cout << "    batteryState     : " << llrf.batteryState << std::endl;
    std::cout << "    position_x       : " << llrf.position_x << std::endl;
    std::cout << "    position_y       : " << llrf.position_y << std::endl;
    std::cout << "    rho              : " << llrf.rho << std::endl;
    std::cout << "    theta            : " << llrf.theta << std::endl;
    std::cout << "    orientation      : " << llrf.orientation << std::endl;
    std::cout << "    angularVelocity  : " << llrf.angularVelocity << std::endl;
    std::cout << "    ballSensor       : " << llrf.ballSensor << std::endl;
    std::cout << "    acceleration_x   : " << llrf.acceleration_x << std::endl;
    std::cout << "    acceleration_y   : " << llrf.acceleration_y << std::endl;
    std::cout << "    velocity_angular : " << llrf.velocity_angular << std::endl;

    std::cout << std::endl;
}

void printRobotFeedback(const roboteam_msgs::RobotFeedback& feedback)
{
    std::cout << "RobotFeedback: " << std::endl;

    std::cout << "    id               : " << feedback.id << std::endl;
    std::cout << "    wheelLeftFront   : " << (feedback.wheelLeftFront ? "1" : "0") << std::endl;
    std::cout << "    wheelRightFront  : " << (feedback.wheelRightFront ? "1" : "0") << std::endl;
    std::cout << "    wheelLeftBack    : " << (feedback.wheelLeftBack ? "1" : "0") << std::endl;
    std::cout << "    wheelRightBack   : " << (feedback.wheelRightBack ? "1" : "0") << std::endl;
    std::cout << "    genevaDriveState : " << (feedback.genevaDriveState ? "1" : "0") << std::endl;
    std::cout << "    batteryState     : " << (feedback.batteryState ? "1" : "0") << std::endl;
    std::cout << "    xPosRobot        : " << feedback.position_x << std::endl;
    std::cout << "    yPosRobot        : " << feedback.position_y << std::endl;
    std::cout << "    rho              : " << feedback.rho << std::endl;
    std::cout << "    theta            : " << feedback.theta << std::endl;
    std::cout << "    orientation      : " << feedback.rotation << std::endl;
    std::cout << "    angularVelocity  : " << feedback.angularVelocity << std::endl;
    std::cout << "    ballSensor       : " << feedback.ballSensor << std::endl;
    std::cout << "    xAcceleration    : " << feedback.acceleration_x << std::endl;
    std::cout << "    yAcceleration    : " << feedback.acceleration_y << std::endl;
    std::cout << "    angularRate      : " << feedback.velocity_angular << std::endl;

    std::cout << std::endl;
}

} // robothub
} // rtt

#pragma clang diagnostic pop