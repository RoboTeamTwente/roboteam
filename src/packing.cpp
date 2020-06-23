#include "packing.h"

#include <math.h>
#include <cassert>
#include <cstdint>
#include <iostream>
#include <memory>
#include "roboteam_utils/Mathematics.h"
#include "roboteam_utils/Vector2.h"
#include "utilities.h"

namespace rtt {
namespace robothub {
/**
 * Creates a low level robot command, i.e. a command from which you can
 * construct a robot packet only from bitshifts, and no other funky angle
 * sin/cos velocity arithmetic. createRobotPacket uses this internally to
 * convert a RobotCommand into something workable.
 */
LowLevelRobotCommand createLowLevelRobotCommand(const proto::RobotCommand& command, proto::World worldOpt, bool isYellow) {
    double kick_chip_power = command.chip_kick_vel();
    double rho = sqrt(command.vel().x() * command.vel().x() + command.vel().y() * command.vel().y());
    double theta = atan2(command.vel().y(), command.vel().x());

    LowLevelRobotCommand llrc{};
    // Units           Represented values
    llrc.id = command.id();                                       // [0, 15]         [0, 15]
    llrc.rho = static_cast<int>(floor(rho * 250));                // [0, 2047] [0, 8.191]
    llrc.theta = static_cast<int>(floor(theta * 1023.5 / M_PI));  // [-1024, 1023]   [-pi, pi>

    llrc.driving_reference = false;        // [0, 1]          {true, false}
    llrc.use_cam_info = false;             // [0, 1]          {true, false}
    llrc.use_angle = command.use_angle();  // [0, 1]          {true, false}
    if (command.use_angle()) {
        llrc.velocity_angular = static_cast<int>(floor(command.w() * (511 / M_PI)));  // [-512, 511]     [-pi, pi]

    } else {
        llrc.velocity_angular = static_cast<int>(floor(command.w() * (511 / (8 * 2 * M_PI))));  // [-512, 511]     [-8*2pi, 8*2pi]
    }
    llrc.debug_info = true;                                                     // [0, 1]          {true, false}
    llrc.do_kick = command.kicker();                                            // [0, 1]          {true, false}
    llrc.do_chip = command.chipper();                                           // [0, 1]          {true, false}
    llrc.kick_chip_forced = command.chip_kick_forced();                         // [0, 1] {true, false}
    llrc.kick_chip_power = static_cast<int>(floor(kick_chip_power * 255 / 6.5));  // [0, 255]        [0, 100]%
    llrc.velocity_dribbler = command.dribbler();                                // [0, 31]        [0, 100]%

    llrc.geneva_drive_state = command.geneva_state();  // [(0)1, 5]       [-2, 2]
    llrc.cam_position_x = 0;                           // [-4096, 4095]   [-10.24, 10.23]
    llrc.cam_position_y = 0;                           // [-4096, 4095]   [-10.24, 10.23]
    llrc.cam_rotation = 0;                             // [-1024, 1023]   [-pi, pi>

    std::shared_ptr<proto::WorldRobot> findBot = utils::getWorldBot(command.id(), isYellow, worldOpt);
    proto::WorldRobot robot;
    if (findBot) {
        robot = *findBot;
        llrc.cam_position_x = 0;  //(int) (robot.pos.x/10.24*4096); // set to 0
                                  // to avoid mystery stop bug
        llrc.cam_position_y = 0;  //(int) (robot.pos.y/10.24*4096); // set to 0
                                  // to avoid mystery stop bug
        llrc.cam_rotation = static_cast<int>(floor(robot.angle() / M_PI * 1024));
        llrc.use_cam_info = true;
    }

    return llrc;
}

bool inRange(int val, int min, int max) { return (min <= val && val <= max); }

/**
 * Checks if the values of the LowLevelRobotCommand are all within the correct
 * range
 * @param llrc : The LowLevelRobotCommand to be checked
 * @returns true if the LowLevelRobotCommand is correct, false otherwise
 */
bool validateRobotPacket(LowLevelRobotCommand llrc) {
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

/* As described in this comment
 * https://roboteamtwente2.slack.com/files/U6CPQLJ6S/F9V330Z2N/packet_proposal.txt
 */
/**
 * Converts a LowLevelRobotCommand into a packet_protocol_message
 * @param llrc : LowLevelRobotCommand to be converted into a
 * packet_protocol_message
 * @returns an optional packed_protocol_message, if the LowLevelRobotCommand can
 * be validated
 */
std::shared_ptr<packed_protocol_message> createRobotPacket(LowLevelRobotCommand llrc) {
    packed_protocol_message byteArr;

    byteArr[0] = static_cast<uint8_t>(  // ID 1 byte
        0b11111111 & llrc.id);

    byteArr[1] = static_cast<uint8_t>(  // All zeros
        0);

    byteArr[2] = static_cast<uint8_t>(  // rho  first byte
        0b11111111 & llrc.rho >> 3);

    byteArr[3] = static_cast<uint8_t>(  // rho last 3 bits | theta first 5 bits
        (0b11100000 & (llrc.rho << 5)) | (0b00011111 & (llrc.theta >> 6)));

    byteArr[4] = static_cast<uint8_t>(  // theta last 6 bits | angle first 2 bits
        (0b11111100 & (llrc.theta << 2)) | (0b00000011 & (llrc.velocity_angular >> 8)));

    byteArr[5] = static_cast<uint8_t>(  // angle last byte
        0b11111111 & llrc.velocity_angular);

    byteArr[6] = static_cast<uint8_t>(  // kick - chip power 1 byte
        0b11111111 & llrc.kick_chip_power);

    byteArr[7] = static_cast<uint8_t>(  // K C F De UC 3 bits of geneva
        (0b10000000 & (llrc.do_kick << 7)) | (0b01000000 & (llrc.do_chip << 6)) | (0b00100000 & (llrc.kick_chip_forced << 5)) | (0b00010000 & (llrc.debug_info << 4)) |
        (0b00001000 & (llrc.use_cam_info << 3)) | (0b00000111 & (llrc.geneva_drive_state)));

    byteArr[8] = static_cast<uint8_t>(  // 5 bits dribble vel | first 3 bits of
                                        // cam rotation
        (0b11111000 & (llrc.velocity_dribbler << 3)) | (0b00000111 & (llrc.cam_rotation >> 8)));

    byteArr[9] = static_cast<uint8_t>(  // Last byte of cam rotation
        0b11111111 & llrc.cam_rotation  // TODO check
    );

    return std::make_shared<packed_protocol_message>(byteArr);
}

/**
 * Converts a LowLevelRobotFeedback into a proto::RobotFeedback
 * TODO this function might be redundant, because LowLevelRobotFeedback and
 * proto::RobotFeedback have exactly the same values
 * TODO consider removing this function, and immediately converting
 * packed_robot_feedback to proto::RobotFeedback
 * @param feedback : LowLevelRobotFeedback to be converted
 * @returns a proto::RobotFeedback object
 */
proto::RobotFeedback toRobotFeedback(LowLevelRobotFeedback feedback) {
    proto::RobotFeedback msg;

    msg.set_id(feedback.id);

    msg.set_xsenscalibrated(feedback.xSensCalibrated);

    msg.set_batterylow(feedback.batteryLow);

    msg.set_ballsensorisworking(feedback.ballSensorWorking);
    msg.set_hasball(feedback.hasBall);
    msg.set_ballpos(feedback.ballPosition);

    msg.set_genevaisworking(feedback.genevaWorking);
    msg.set_genevastate(feedback.genevaState);

    msg.set_x_vel((feedback.rho * 0.004) * cos(feedback.theta * 0.00307));
    msg.set_y_vel((feedback.rho * 0.004) * sin(feedback.theta * 0.00307));
    msg.set_yaw(feedback.angle * 0.00614);

    msg.set_haslockedwheel(feedback.hasLockedWheel);
    msg.set_signalstrength(feedback.signalStrength);

    return msg;
}

/**
 * Converts the bytes received from the robot into a LowLevelRobotFeedback
 * object
 * @param bitsnbytes The bytes from the robot
 * @returns a LowLevelRobotFeedback struct
 */
LowLevelRobotFeedback createRobotFeedback(packed_robot_feedback bits) {
    struct LowLevelRobotFeedback feedback {};

    feedback.id = bits[0];

    feedback.xSensCalibrated = (0b10000000 & bits[1]) >> 7;
    feedback.batteryLow = (0b01000000 & bits[1]) >> 6;
    feedback.ballSensorWorking = (0b00100000 & bits[1]) >> 5;
    feedback.hasBall = (0b00010000 & bits[1]) >> 4;
    feedback.ballPosition = 0b00001111 & bits[1];

    feedback.genevaWorking = (0b10000000 & bits[2]) >> 7;
    feedback.genevaState = 0b01111111 & bits[2];

    feedback.rho = (0b11111111 & bits[3]) << 3;
    feedback.rho |= (0b11100000 & bits[4]) >> 5;

    feedback.angle = (0b00011111 & bits[4]) << 5;
    feedback.angle |= (0b11111000 & bits[5]) >> 3;

    feedback.theta = (0b00000111 & bits[5]) << 8;
    feedback.theta |= 0b11111111 & bits[6];

    feedback.hasLockedWheel = (0b10000000 & bits[7]) >> 7;
    feedback.signalStrength = 0b01111111 & bits[7];

    return feedback;
}

void printRobotCommand(const proto::RobotCommand& cmd) {
    /**
     * C-style casts can fail runtime and return garbage values, but not only
     * that, they're way more dangerous
     *
     * const int* const x = new int{ 500 };
     * (int*)x = 700; // ignores CV qualifiers, acts like a const_cast which is
     * heavily frowned upon
     * // reading x is now UB
     * delete x;
     *
     */
    std::cout << "RobotCommand: " << std::endl;

    std::cout << "    id             : " << cmd.id() << std::endl;
    std::cout << "    active         : " << static_cast<int>(cmd.active()) << std::endl;
    std::cout << "    x_vel          : " << cmd.vel().x() << std::endl;
    std::cout << "    y_vel          : " << cmd.vel().y() << std::endl;
    std::cout << "    w              : " << cmd.w() << std::endl;
    std::cout << "    use_angle      : " << static_cast<int>(cmd.use_angle()) << std::endl;
    std::cout << "    dribbler       : " << static_cast<int>(cmd.dribbler()) << std::endl;
    std::cout << "    kicker         : " << static_cast<int>(cmd.kicker()) << std::endl;
    std::cout << "    kicker_forced  : " << static_cast<int>(cmd.chip_kick_forced()) << std::endl;
    std::cout << "    kicker_vel     : " << cmd.chip_kick_vel() << std::endl;
    std::cout << "    chipper        : " << static_cast<int>(cmd.chipper()) << std::endl;
    std::cout << "    chipper_forced : " << static_cast<int>(cmd.chip_kick_forced()) << std::endl;
    std::cout << "    chipper_vel    : " << cmd.chip_kick_vel() << std::endl;
    std::cout << "    geneva_state   : " << cmd.geneva_state() << std::endl;

    std::cout << std::endl;
}

void printLowLevelRobotCommand(const LowLevelRobotCommand& llrc) {
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

void printLowLevelRobotFeedback(const LowLevelRobotFeedback& llrf) {
    std::cout << "LowLevelRobotFeedback: " << std::endl;

    std::cout << "    id               : " << llrf.id << std::endl;
    std::cout << "    xsensCalibrated  : " << llrf.xSensCalibrated << std::endl;
    std::cout << "    batteryLow       : " << (llrf.batteryLow ? "1" : "0") << std::endl;
    std::cout << "    bs_Working       : " << (llrf.ballSensorWorking ? "1" : "0") << std::endl;
    std::cout << "    bs_hasBall       : " << (llrf.hasBall ? "1" : "0") << std::endl;
    std::cout << "    bs_BallPosition  : " << (llrf.ballPosition ? "1" : "0") << std::endl;
    std::cout << "    geneva_Working   : " << (llrf.genevaWorking ? "1" : "0") << std::endl;
    std::cout << "    geneva_State     : " << llrf.genevaState << std::endl;
    std::cout << "    rho              : " << llrf.rho << std::endl;
    std::cout << "    theta            : " << llrf.theta << std::endl;
    std::cout << "    angle            : " << llrf.angle << std::endl;
    std::cout << "    hasLockedWheel   : " << llrf.hasLockedWheel << std::endl;
    std::cout << "    signalStrength   : " << llrf.signalStrength << std::endl;

    std::cout << std::endl;
}

void printRobotFeedback(const proto::RobotFeedback& feedback) {
    std::cout << "RobotFeedback.proto: " << std::endl;

    std::cout << "    id               : " << feedback.id() << std::endl;
    std::cout << "    xsensCalibrated  : " << feedback.xsenscalibrated() << std::endl;
    std::cout << "    batteryLow       : " << (feedback.batterylow() ? "1" : "0") << std::endl;
    std::cout << "    bs_Working       : " << (feedback.ballsensorisworking() ? "1" : "0") << std::endl;
    std::cout << "    bs_hasBall       : " << (feedback.hasball() ? "1" : "0") << std::endl;  // not sure what hasball is
    // if hasball is a boolean then you can just print it it'll conver to 1 / 0
    // alreayd, if you want it to be `true` or `false` then std::cout <<
    // std::boolalpha first
    std::cout << "    bs_BallPosition  : " << feedback.ballpos() << std::endl;
    std::cout << "    geneva_Working   : " << (feedback.genevaisworking() ? "1" : "0") << std::endl;
    std::cout << "    geneva_State     : " << feedback.genevastate() << std::endl;
    std::cout << "    x_vel            : " << feedback.x_vel() << std::endl;
    std::cout << "    y_vel            : " << feedback.y_vel() << std::endl;
    std::cout << "    yaw              : " << feedback.yaw() << std::endl;
    std::cout << "    hasLockedWheel   : " << feedback.haslockedwheel() << std::endl;
    std::cout << "    signalStrength   : " << feedback.signalstrength() << std::endl;

    std::cout << std::endl;
}

}  // namespace robothub
}  // namespace rtt