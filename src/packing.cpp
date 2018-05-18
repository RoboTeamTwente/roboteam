#include "roboteam_robothub/packing.h"
#include "roboteam_robothub/packet_tester.h"

#include <array>
#include <cassert>
#include <cstdint>
#include <iostream>
#include <memory>
#include <boost/optional.hpp>
#include <math.h>
#include <ros/ros.h>

namespace b = boost;

#include "roboteam_msgs/RobotCommand.h"
#include "roboteam_msgs/RobotFeedback.h"
#include "roboteam_msgs/World.h"
#include "roboteam_utils/Vector2.h"
#include "roboteam_utils/Math.h"

namespace rtt {

    namespace {

        uint8_t normalizeToByte(double const forceRaw, double const max, uint8_t normMax) {
            if (forceRaw >= max) {
                return normMax;
            } else if (forceRaw < 0) {
                return 0;
            } else {
                return forceRaw / max * normMax;
            }
        }

        // Copy of getWorldBot() because I don't want to pull in tactics as a dependency. If this function is moved to utils
        // we can use that
        boost::optional<roboteam_msgs::WorldRobot>
        getWorldBot(unsigned int id, bool ourTeam, const roboteam_msgs::World &world) {
            std::vector<roboteam_msgs::WorldRobot> bots = ourTeam ? world.us : world.them;
            for (const auto &bot : bots) {
                if (bot.id == id) {
                    return boost::optional<roboteam_msgs::WorldRobot>(bot);
                }
            }
            return boost::none;
        }
    } // anonymous namespace

    /**
     * Creates a low level robot command, i.e. a command from which you can construct a robot packet
     * only from bitshifts, and no other funky angle sin/cos velocity arithmetic. createRobotPacket
     * uses this internally to convert a RobotCommand into something workable.
     */
    LowLevelRobotCommand createLowLevelRobotCommand(const roboteam_msgs::RobotCommand& command, const b::optional<roboteam_msgs::World>& worldOpt) {

        double kick_chip_power = fmax(command.kicker_vel, command.chipper_vel);
        double rho = sqrt(command.x_vel * command.x_vel + command.y_vel * command.y_vel);
        double theta = command.x_vel == 0 ? 0 : atan(command.y_vel / command.x_vel);

        LowLevelRobotCommand llrc {};
                                                                                   // Units           Represented values
        llrc.id                 = command.id;                                      // [0, 15]         [0, 15]
        llrc.rho                = (int)floor(rho * 250);                           // [0, 2047]       [0, 8.191]
        llrc.theta              = (int)floor(theta * (1024 / M_PI));               // [-1024, 1023]   [-pi, pi>
        llrc.driving_reference  = false;                                           // [0, 1]          {true, false}
        llrc.use_cam_info       = false;                                           // [0, 1]          {true, false}
        llrc.velocity_angular   = (int)floor(command.w * (511 / (8 * 2*M_PI)));    // [-512, 511]     [-8*2pi, 8*2pi]
        llrc.debug_info         = true;                                            // [0, 1]          {true, false}
        llrc.do_kick            = command.kicker;                                  // [0, 1]          {true, false}
        llrc.do_chip            = command.chipper;                                 // [0, 1]          {true, false}
        llrc.kick_chip_forced   = command.kicker_forced || command.chipper_forced; // [0, 1]          {true, false}
        llrc.kick_chip_power    = (int)floor(kick_chip_power * 255 / 100.0);       // [0, 255]        [0, 100]%
        llrc.velocity_dribbler  = 0;//(int)floor(command.dribbler * (100 / 255));  // [0, 255]        [0, 100]%
        llrc.geneva_drive_state = 0;//command.geneva_state - 3;                    // [(0)1, 5]       [-2, 2]
        llrc.cam_position_x     = 0;                                               // [-4096, 4095]   [-10.24, 10.23]
        llrc.cam_position_y     = 0;                                               // [-4096, 4095]   [-10.24, 10.23]
        llrc.cam_rotation       = 0;                                               // [-1024, 1023]   [-pi, pi>

		if(worldOpt) {
			try {
				roboteam_msgs::WorldRobot robot = worldOpt->us.at(command.id);
				llrc.cam_position_x = (int) (robot.pos.x / 10.24 * 4096);
				llrc.cam_position_y = (int) (robot.pos.y / 10.24 * 4096);
				llrc.cam_rotation   = (int) floor(robot.angle / M_PI * 1024);
				llrc.use_cam_info   = true;
			}catch(std::out_of_range e){
				ROS_WARN_STREAM("[createLowLevelRobotCommand] Robot " << command.id << " not present in World! Not adding camera data");
			}catch(std::exception e){
				ROS_WARN_STREAM("[createLowLevelRobotCommand] Something went wrong while adding camera data for robot " << command.id << " : " << e.what());
			}
		}

        return llrc;
    }

    boost::optional<packed_protocol_message> createRobotPacket(roboteam_msgs::RobotCommand const &command, b::optional<roboteam_msgs::World> const &world){
        auto llcommand = createLowLevelRobotCommand(command, world);
        return createRobotPacket(llcommand);
    }

    bool inRange(int val, int min, int max){
        return (min <= val && val <= max);
    }

    bool validateRobotPacket(LowLevelRobotCommand llrc){

        // inRange(val, min, max) is inclusive!;
        // inRange(-5, -5, 10) -> true; // inRange(14, -5, 10) -> false;
        // inRange(10, -5, 10) -> true; // inRange(-8, -5, 10) -> false;

        // Holds if all the values of the command are in range
        bool valuesInRange = true;

        valuesInRange &= inRange(llrc.id, 0, 15);
        valuesInRange &= inRange(llrc.rho, 0, 2047);
        valuesInRange &= inRange(llrc.theta, -1024, 1023);
        valuesInRange &= inRange(llrc.velocity_angular, 0, 511);
        valuesInRange &= inRange(llrc.kick_chip_power, 0, 255);
        valuesInRange &= inRange(llrc.velocity_dribbler, 0, 255);
        valuesInRange &= inRange(llrc.geneva_drive_state, 0, 4);
        valuesInRange &= inRange(llrc.cam_position_x, -4096, 4095);
        valuesInRange &= inRange(llrc.cam_position_y, -4096, 4095);
        valuesInRange &= inRange(llrc.cam_rotation, -1024, 1023);

        return valuesInRange;
    }

    /* As described in this comment https://roboteamtwente2.slack.com/files/U6CPQLJ6S/F9V330Z2N/packet_proposal.txt */
    boost::optional<packed_protocol_message> createRobotPacket(LowLevelRobotCommand llrc){

        // Values are automatically limited in the code below, but returning boost::none is a good idea nonetheless.
        if(!validateRobotPacket(llrc)){
            ROS_WARN_STREAM("LowLevelRobotCommand is not valid");
            return boost::none;
        }

        //Might still be wrong because certain people can't make their minds up
        packed_protocol_message byteArr;

        // Static cast truncates to the last 8 bits
        // is implementation defined though.

        byteArr[0] = static_cast<uint8_t>(  // aaaaabbb
            (0b11111000 & (llrc.id << 3))                   //aaaaa000   5 bits; bits  4-0 to 7-3
          | (0b00000111 & (llrc.rho >> 8))                  //00000bbb  11 bits; bits 10-8 to 2-0
        );

        byteArr[1] = static_cast<uint8_t>(  // bbbbbbbb
            llrc.rho                                        //bbbbbbbb  11 bits; bits  7-0 to 7-0
        );

        byteArr[2] = static_cast<uint8_t>(  // cccccccc
            llrc.theta >> 3                                 // cccccccc 11 bits; bits 10-8 to 7-0
        );

        byteArr[3] = static_cast<uint8_t>(  // cccde0fg
            (0b11100000 & (llrc.theta << 5)) |              // ccc00000 11 bits; bits  2-0 to 7-5
            (0b00010000 & (llrc.driving_reference << 4)) |  // 000d0000  1 bit ; bit     0 to   4
            (0b00001000 & (llrc.use_cam_info) << 3) |       // 0000e000  1 bit ; bit     0 to   3
            (0b00000001 & (llrc.velocity_angular >> 8))     // 0000000g  9 bits; bit     8 to   0
        );

        byteArr[4] = static_cast<uint8_t>(  // gggggggg
            llrc.velocity_angular                           // gggggggg  8 bits; bits  7-0 to 7-0
        );

        byteArr[5] = static_cast<uint8_t>(  // 0000hijk
            (0b00001000 & (llrc.debug_info << 3)) |         // 0000h000  1 bit ; bit     0 to   3
            (0b00000100 & (llrc.do_kick << 2)) |            // 00000i00  1 bit ; bit     0 to   2
            (0b00000010 & (llrc.do_chip << 1)) |            // 000000j0  1 bit ; bit     0 to   1
            (0b00000001 & (llrc.kick_chip_forced))     		// 0000000k  1 bit ; bit     0 to   0
        );

        byteArr[6] = static_cast<uint8_t>(  // mmmmmmmm
            llrc.kick_chip_power                            // mmmmmmmm  8 bits; bits  7-0 to 7-0
        );

        byteArr[7] = static_cast<uint8_t>(  // nnnnnnnn
            llrc.velocity_dribbler                          // nnnnnnnn  8 bits; bits  7-0 to 7-0
        );

        byteArr[8] = static_cast<uint8_t>(  // pppqqqqq
            (0b11100000 & (llrc.geneva_drive_state << 5)) | // ppp00000  3 bits; bits  2-0 to 7-5
            (0b00011111 & (llrc.cam_position_x >> 8 ))      // 000qqqqq 13 bits; bits 12-8 to 4-0
        );

        byteArr[9] = static_cast<uint8_t>(  // qqqqqqqq
            llrc.cam_position_x                             // qqqqqqqq 13 bits; bits  7-0 to 7-0
        );

        byteArr[10] = static_cast<uint8_t>( // rrrrrrrr
            llrc.cam_position_y >> 5                        // rrrrrrrr 13 bits; bits 12-5 to 7-0
        );

        byteArr[11] = static_cast<uint8_t>( // rrrrrsss
            (0b11111000 & (llrc.cam_position_y << 3)) |     // rrrrr000 13 bits; bits  4-0 to 7-3
            (0b00000111 & (llrc.cam_rotation >> 8))         // 00000sss 11 bits; bits 10-8 to 2-0
        );

        byteArr[12] = static_cast<uint8_t>( // ssssssss
            llrc.cam_rotation                               // ssssssss 11 bits; bits  7-0 to 7-0
        );

        return boost::optional<packed_protocol_message>(byteArr);
    }




    roboteam_msgs::RobotFeedback toRobotFeedback(LowLevelRobotFeedback feedback){
        roboteam_msgs::RobotFeedback msg;

        msg.id = feedback.id;
        msg.wheelLeftFront  = feedback.wheelLeftFront ;
        msg.wheelRightFront = feedback.wheelRightFront;
        msg.wheelLeftBack   = feedback.wheelLeftBack;
        msg.wheelRightBack  = feedback.wheelRightBack;

        msg.genevaDriveState = feedback.genevaDriveState;
        msg.batteryState     = feedback.batteryState;

        msg.position_x = feedback.position_x;
        msg.position_y = feedback.position_y;

        msg.rho     = feedback.rho;
        msg.theta   = feedback.theta;
        msg.rotation= feedback.orientation;

        msg.angularVelocity = feedback.angularVelocity;
        msg.ballSensor      = feedback.ballSensor;

        msg.acceleration_x = feedback.acceleration_x;
        msg.acceleration_y = feedback.acceleration_y;
        msg.velocity_angular = feedback.velocity_angular;

        return msg;
    }









/*
 * For the Robot ACKs we use the following packet definition
 *
 *skipping some characters for better readability

		Character   Description                 Values          Represented values    Units       Bits    Comment
		a           Robot ID                    [0,15]          [0,31]                -              5    -
		b           Left front wheel state      [0,1]           {true,false}          -              1    Indicates whether the left front wheel functions
		c           Right front wheel state     [0,1]           {true,false}          -              1    Indicates whether the right front wheel functions
		d           Left back wheel state       [0,1]           {true,false}          -              1    Indicates whether the left back wheel functions
		e           Right back wheel state      [0,1]           {true,false}          -              1    Indicates whether the right back wheel functions
		f           Geneva drive state          [0,1]           {true,false}          -              1    Indicates whether the Geneva drive functions
		g           Battery state               [0,1]           {true,false}          -              1    States whether the battery is nearing depletion
		h           x position robot            [-4096,4095]    [-1024,1023]          0.25cm        13    -
		k           y position robot            [-4096,4095]    [-1024,1023]          0.25cm        13    -
		m           rho            				[-1024,1023]    [?,?]		          	            11    Magnitude of the robot velocity vector
		o           theta           			[-1024,1023]    [?,?]                               11    Angle of the robot velocity vector
		p           Orientation                 [-1024,1023]    [-pi,pi>              0.00307rad    11    Angle of the facing direction. 2048 possible angles. Intervals of ~0.00307 rad
		q           Angular velocity            [-1024,1023]    [?,?]                 0.049rad/s?   11
		s           Ball sensor                 [0,128]         {?}			          -              7    Can be used to specify where on the dribbler the ball is located. For now a non-zero value represents the presence of the ball

		Extra
		t           Acceleration x              [0, 4294967295] [0, 32 bit float]     m/s/s         32    -
		u           Acceleration y              [0, 4294967295] [0, 32 bit float]     m/s/s         32    -
		v           Angular rate                [0, 4294967295] [0, 32 bit float]     m/s/s         32    raw angular velocity from xsense


	===== Packet received from the robot =====
		Byte      Config
		 0        aaaaabcd
		 1        efghhhhh
		 2        hhhhhhhh
		 3        kkkkkkkk
		 4        kkkkkmmm
		 5        mmmmmmmm
		 6        oooooooo
		 7        oooppppp
		 8        ppppppqq
		 9        qqqqqqqq
		10        qsssssss

		Extra
		11        tttttttt
		12        tttttttt
		13        tttttttt
		14        tttttttt
		15        uuuuuuuu
		16        uuuuuuuu
		17        uuuuuuuu
		18        uuuuuuuu
		19        vvvvvvvv
		20        vvvvvvvv
		21        vvvvvvvv
		22        vvvvvvvv

 */

	/**
	 * Converts the bytes received from the robot into a LowLevelRobotFeedback object
	 * @param bitsnbytes The bytes from the robot
	 * @returns a LowLevelRobotFeedback struct
	 */
    LowLevelRobotFeedback createRobotFeedback(packed_robot_feedback bitsnbytes){
        struct LowLevelRobotFeedback feedback;

        int offset = 0;

        // 0 aaaaabcd
        feedback.id = bitsnbytes[offset+0] >> 3;
        feedback.wheelLeftFront = (bitsnbytes[offset+0] & 4) == 4;
        feedback.wheelRightFront= (bitsnbytes[offset+0] & 2) == 2;
        feedback.wheelLeftBack  = (bitsnbytes[offset+0] & 1) == 1;

        // 1 efghhhhh
        feedback.wheelRightBack     = (bitsnbytes[offset+1] & (1<<7)) == (1<<7);
        feedback.genevaDriveState   = (bitsnbytes[offset+1] & (1<<6)) == (1<<6);
        feedback.batteryState       = (bitsnbytes[offset+1] & (1<<5)) == (1<<5);
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
        feedback.orientation   |= bitsnbytes[offset+8] >> 2;
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

	void printRobotCommand(const roboteam_msgs::RobotCommand& cmd){
		std::cout << "RobotCommand: " << std::endl;

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

		std::cout << std::endl;
	}

	void printLowLevelRobotCommand(const LowLevelRobotCommand& llrc){
		std::cout << "LowLevelRobotCommand: " << std::endl;

		std::cout << "    id                 : " << llrc.id                 << std::endl;
		std::cout << "    rho                : " << llrc.rho                << std::endl;
		std::cout << "    theta              : " << llrc.theta              << std::endl;
		std::cout << "    driving_reference  : " << llrc.driving_reference  << std::endl;
		std::cout << "    use_cam_info       : " << llrc.use_cam_info       << std::endl;
		std::cout << "    velocity_angular   : " << llrc.velocity_angular   << std::endl;
		std::cout << "    debug_info         : " << llrc.debug_info         << std::endl;
		std::cout << "    do_kick            : " << llrc.do_kick            << std::endl;
		std::cout << "    do_chip            : " << llrc.do_chip            << std::endl;
		std::cout << "    kick_chip_forced   : " << llrc.kick_chip_forced   << std::endl;
		std::cout << "    kick_chip_power    : " << llrc.kick_chip_power    << std::endl;
		std::cout << "    velocity_dribbler  : " << llrc.velocity_dribbler  << std::endl;
		std::cout << "    geneva_drive_state : " << llrc.geneva_drive_state << std::endl;
		std::cout << "    cam_position_x     : " << llrc.cam_position_x     << std::endl;
		std::cout << "    cam_position_y     : " << llrc.cam_position_y     << std::endl;
		std::cout << "    cam_rotation       : " << llrc.cam_rotation       << std::endl;

		std::cout << std::endl;
	}

	void printLowLevelRobotFeedback(const LowLevelRobotFeedback& llrf){
		std::cout << "LowLevelRobotFeedback: " << std::endl;

		std::cout << "    id               : " << llrf.id               << std::endl;
		std::cout << "    wheelLeftFront   : " << llrf.wheelLeftFront   << std::endl;
		std::cout << "    wheelRightFront  : " << llrf.wheelRightFront  << std::endl;
		std::cout << "    wheelLeftBack    : " << llrf.wheelLeftBack    << std::endl;
		std::cout << "    wheelRightBack   : " << llrf.wheelRightBack   << std::endl;
		std::cout << "    genevaDriveState : " << llrf.genevaDriveState << std::endl;
		std::cout << "    batteryState     : " << llrf.batteryState     << std::endl;
		std::cout << "    position_x       : " << llrf.position_x       << std::endl;
		std::cout << "    position_y       : " << llrf.position_y       << std::endl;
		std::cout << "    rho              : " << llrf.rho              << std::endl;
		std::cout << "    theta            : " << llrf.theta            << std::endl;
		std::cout << "    orientation      : " << llrf.orientation      << std::endl;
		std::cout << "    angularVelocity  : " << llrf.angularVelocity  << std::endl;
		std::cout << "    ballSensor       : " << llrf.ballSensor       << std::endl;
		std::cout << "    acceleration_x   : " << llrf.acceleration_x   << std::endl;
		std::cout << "    acceleration_y   : " << llrf.acceleration_y   << std::endl;
		std::cout << "    velocity_angular : " << llrf.velocity_angular << std::endl;

		std::cout << std::endl;
	}

	void printRobotFeedback(const roboteam_msgs::RobotFeedback& feedback){
		std::cout << "RobotFeedback: " << std::endl;

		std::cout << "    id               : " << feedback.id               << std::endl;
		std::cout << "    wheelLeftFront   : " <<(feedback.wheelLeftFront   ? "1" : "0") << std::endl;
		std::cout << "    wheelRightFront  : " <<(feedback.wheelRightFront  ? "1" : "0") << std::endl;
		std::cout << "    wheelLeftBack    : " <<(feedback.wheelLeftBack    ? "1" : "0") << std::endl;
		std::cout << "    wheelRightBack   : " <<(feedback.wheelRightBack   ? "1" : "0") << std::endl;
		std::cout << "    genevaDriveState : " <<(feedback.genevaDriveState ? "1" : "0") << std::endl;
		std::cout << "    batteryState     : " <<(feedback.batteryState     ? "1" : "0") << std::endl;
		std::cout << "    xPosRobot        : " << feedback.position_x       << std::endl;
		std::cout << "    yPosRobot        : " << feedback.position_y       << std::endl;
		std::cout << "    rho              : " << feedback.rho              << std::endl;
		std::cout << "    theta            : " << feedback.theta            << std::endl;
		std::cout << "    orientation      : " << feedback.rotation         << std::endl;
		std::cout << "    angularVelocity  : " << feedback.angularVelocity  << std::endl;
		std::cout << "    ballSensor       : " << feedback.ballSensor       << std::endl;
		std::cout << "    xAcceleration    : " << feedback.acceleration_x   << std::endl;
		std::cout << "    yAcceleration    : " << feedback.acceleration_y   << std::endl;
		std::cout << "    angularRate      : " << feedback.velocity_angular << std::endl;

		std::cout << std::endl;
	}










//    std::string byteToBinary(uint8_t const &byte) {
//        std::string byteStr = "";
//        for (int i = 0; i < 8; i++) {
//            if ((byte & (1 << i)) == (1 << i)) {
//                byteStr = "1" + byteStr;
//            } else {
//                byteStr = "0" + byteStr;
//            }
//        }
//
//        return byteStr;
//    }



} // rtt

//bool operator==(const rtt::LowLevelRobotCommand &lhs, const rtt::LowLevelRobotCommand &rhs) {
//    return false;
//            lhs.id == rhs.id &&
//            lhs.robot_vel == rhs.robot_vel &&
//            lhs.ang == rhs.ang &&
//            lhs.rot_cclockwise == rhs.rot_cclockwise &&
//            lhs.w == rhs.w &&
//            lhs.punt_power == rhs.punt_power &&
//            lhs.do_kick == rhs.do_kick &&
//            lhs.do_chip == rhs.do_chip &&
//            lhs.forced == rhs.forced &&
//            lhs.dribble_cclockwise == rhs.dribble_cclockwise &&
//            lhs.dribble_vel == rhs.dribble_vel;
//}
//
//bool operator!=(const rtt::LowLevelRobotCommand &lhs, const rtt::LowLevelRobotCommand &rhs) {
//    return !(lhs == rhs);
//}
