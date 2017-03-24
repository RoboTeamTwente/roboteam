#include "roboteam_robothub/packing.h"

#include <array>
#include <cassert>
#include <cstdint>
#include <iostream>
#include <memory>
#include <boost/optional.hpp>

#include "roboteam_msgs/RobotCommand.h"
#include "roboteam_utils/Vector2.h"

namespace rtt {

/**
 * Command packet format, inspired by the (old?) RoboJackets protocol.
 *
 * Byte     Config      Description
 * 1        aaaabbbb    aaaa: Robot ID, bbbb: Robot velocity
 * 2        bbbbbbbb    bbbbbbbb: Robot velocity, 0 - 8191 (mm/s)
 * 3        bccccccc    b: Robot velocity, cccccccc: Moving direction, resolution 2 * pi / 512
 * 4        cc00deee    cc: Moving direction, d: Rotation direction, true is counter clockwise
 * 5        eeeeeeee    eeeeeeeeeee: Angular velocity, 0-2047 (deg/s)
 * 6        ffffffff    ffffffff: Kick force, 0 - 255
 * 7        0ghijkkk    g: whether or not to kick
 *                      h: whether or not to chip
 *                      i: forced or not
 *                      j: counterclockwise dribbler. Counterclockwise means spinning in a way s.t. the ball does not spin toward the robot (i.e. with the robot facing towards the right).
 *                      kkk: dribbler speed, 0 - 7
 */

/**
 * Packet response format, inspired by Hans, Jim & Bob.
 *
 * Byte     Config      Description
 *                      Robot ID
 *                      ACK or NACK bit (whether or not the command was actually
 *                      dispatched from the base station)
 *                      Battery level
 *                      Ball sensor (binary or continuous)
 *                      Bit for debug information
 *
 *                      If bit for debug is one, extra info is also sent:
 *                      Gyroscopoe info
 *                      Accelerometer info
 *
 *                      Copy of command packet, possibly only when requested.
 */

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

} // anonymous namespace

/**
 * Creates a low level robot command, i.e. a command from which you can construct a robot packet
 * only from bitshifts, and no other funky angle sin/cos velocity arithmetic. createRobotPacket
 * uses this internally to convert a RobotCommand into something workable.
 */
LowLevelRobotCommand createLowLevelRobotCommand(roboteam_msgs::RobotCommand const & command) {
    using roboteam_msgs::RobotCommand;
    
    //////////////////////////////
    // Calculate robot velocity //
    //////////////////////////////
    Vector2 velocityVec(command.x_vel, command.y_vel);

    int robot_vel = velocityVec.length() * 1000;
    if (robot_vel > PACKET_MAX_ROBOT_VEL) {
        robot_vel = PACKET_MAX_ROBOT_VEL;
    }

    ///////////////////////////
    // Calculate robot angle //
    ///////////////////////////
    double rawAng = velocityVec.angle();
    
    // Domain of rawAng == [-pi, +pi]. If it is below zero we must put it
    // in the positive domain.
    if (rawAng < 0) {
        rawAng += M_PI * 2;
    }

    int ang = rawAng / (M_PI * 2) * 512;

    if (ang > PACKET_MAX_ANG) {
        ang = PACKET_MAX_ANG;
    }

    //////////////////////////////////////
    // Calculate w & rotation direction //
    //////////////////////////////////////
    // Calculate w & if to rotate clockwise or counter-clockwise
    // If w is positive it's counter clockwise
    bool rot_cclockwise = command.w > 0;
    double rawW = command.w;
    if (rawW < 0) {
        rawW = -rawW;
    }
    // w is in deg/s
    int w = rawW / M_PI * 180;

    if (w > 2047) w = 2047;
    if (w < 0) w = 0;
    
    //////////////////////////
    // Calculate kick_force //
    //////////////////////////
    // uint8_t kick_force = normalizeToByte(command.kicker_vel, RobotCommand::MAX_KICKER_VEL);
    // uint8_t dribbler_force = normalizeToByte(command.chipper_vel, RobotCommand::MAX_CHIPPER_VEL);
    
    uint8_t kick_force = 0;
    uint8_t chip_force = 0;
    bool do_chip = false;
    bool do_kick = false;
    bool do_forced = false;

    if (command.kicker == true) {
        do_kick = true;
        kick_force = normalizeToByte(command.kicker_vel, RobotCommand::MAX_KICKER_VEL, 255);
        do_forced = command.kicker_forced;
    } else if (command.chipper == true) {
        do_chip = true;
        chip_force = normalizeToByte(command.chipper_vel, RobotCommand::MAX_CHIPPER_VEL, 255);
        do_forced = command.chipper_forced;
    }

    bool dribble_cclockwise = false;
    uint8_t dribble_vel = 0;
    if (command.dribbler) {
        dribble_vel = PACKET_MAX_DRIBBLE_VEL;
    }

    ///////////////////////////////////////
    // Construct low level robot command //
    ///////////////////////////////////////

    LowLevelRobotCommand llcommand;
    llcommand.id = command.id;
    llcommand.robot_vel = robot_vel;
    llcommand.ang = ang;
    llcommand.rot_cclockwise = rot_cclockwise;
    llcommand.w = w;
    llcommand.punt_power = std::max(kick_force, chip_force);
    llcommand.do_kick = do_kick;
    llcommand.do_chip = do_chip;
    llcommand.forced = do_forced;
    llcommand.dribble_cclockwise = dribble_cclockwise;
    llcommand.dribble_vel = dribble_vel;

    // std::cout << "[RobotHub] ------------------------------------------\n";
    // std::cout << "[RobotHub] About to send the following w to the motor: " << llcommand.w << "\n";
    // std::cout << "[RobotHub] rot_cclockwise: " << (int) llcommand.rot_cclockwise << "\n";

    return llcommand;
}

/**
 * TODO: Measure our highest kicking power and use that to scale kick_vel!
 */
boost::optional<packed_protocol_message> createRobotPacket(roboteam_msgs::RobotCommand const & command) {
    auto llcommand = createLowLevelRobotCommand(command);

    return createRobotPacket(
            llcommand.id,
            llcommand.robot_vel,
            llcommand.ang,
            llcommand.rot_cclockwise,
            llcommand.w,
            llcommand.punt_power,
            llcommand.do_kick,
            llcommand.do_chip,
            llcommand.forced,
            llcommand.dribble_cclockwise,
            llcommand.dribble_vel
            );
}

boost::optional<packed_protocol_message> createRobotPacket(LowLevelRobotCommand llrc) {
    return createRobotPacket(
            llrc.id,
            llrc.robot_vel,
            llrc.ang,
            llrc.rot_cclockwise,
            llrc.w,
            llrc.punt_power,
            llrc.do_kick,
            llrc.do_chip,
            llrc.forced,
            llrc.dribble_cclockwise,
            llrc.dribble_vel
            );
}

/**
 * All mentioned ranges are inclusive. If the specified ranges are violated,
 * the function returns boost::none.
 *
 * \param id Must be between 0 and 15
 * \param robot_vel Must be between 0 and 4095 (mm/s)
 * \param ang Moving direction. Must be between 0 and 511. Resolution
 *          is 2*pi/512
 * \param rot_cclockwise True if rotation is counter clockwise
 * \param w Angular velocity. Must be between 0 and 2047
 * \param kick_force Naturally between 0 and 255
 * \param do_kick Whether or not a chip or kick should be done
 * \param chip True if the robot should chip, otherwise a kick will be done
 * \param forced If true robot will not wait until it's close to the ball
 * \param dribble_cclockwise If true dribbler will spin counter clockwise
 * \param dribble_vel Must be between 0 and 7.
 */
boost::optional<packed_protocol_message> createRobotPacket(int32_t id, int32_t robot_vel, int32_t ang,
                                                         bool rot_cclockwise, int32_t w, uint8_t punt_power,
                                                         bool do_kick, bool do_chip, bool forced,
                                                         bool dribble_cclockwise, uint8_t dribble_vel) {
    if (!((id >= 0 && id < 16)
            && (robot_vel >= 0 && robot_vel < 8192)
            && (ang >= 0 && ang < 512)
            && (w >= 0 && w < 2048)
            && (dribble_vel >= 0 && dribble_vel < 8))) {
        return boost::none;
    }

    packed_protocol_message byteArr;

    // First nibble is the robot id
    // Second nibble are the third nibble of robot velocity
    byteArr[0] = static_cast<uint8_t>(((id & 15) << 4) | ((robot_vel >> 9) & 15));
    // First and second nibble of robot velocity
    byteArr[1] = static_cast<uint8_t>(robot_vel >> 1);
    // Second to ninth bit of moving direction
    byteArr[2] = static_cast<uint8_t>(robot_vel & 1) << 7
                | static_cast<uint8_t>(ang >> 2);
    // First bit of moving direction
    // Then bit that designates clockwise rotation or not
    // Last three bits of angular velocity
    byteArr[3] = static_cast<uint8_t>((ang & 3) << 6) 
                | static_cast<uint8_t>(rot_cclockwise << 3)
                | static_cast<uint8_t>((w >> 8) & 7);
    // First two nibbles of angular velocity
    byteArr[4] = static_cast<uint8_t>(w);
    // Just plug in the byte of kick-force
    byteArr[5] = punt_power;
    // gggg = 0, 0, chip = 1 kick = 0, forced = 1 normal = 0
    // First the chip and forced bools, then a bool that designates
    // a clockwise dribbler, and then three bits to designate dribble velocity
    byteArr[6] = static_cast<uint8_t>(do_kick << 6)
                    | static_cast<uint8_t>(do_chip << 5)
                    | static_cast<uint8_t>(forced << 4)
                    | static_cast<uint8_t>(dribble_cclockwise << 3)
                    | static_cast<uint8_t>(dribble_vel & 7);

    return boost::optional<packed_protocol_message>(byteArr);
}

std::string byteToBinary(uint8_t byte) {
    std::string byteStr = "";
    for (int i = 0; i < 8; i++) {
        if ((byte & (1 << i)) == (1 << i)) {
            byteStr = "1" + byteStr;
        } else {
            byteStr = "0" + byteStr;
        }
    }

    return byteStr;
}

} // rtt

bool operator==(const rtt::LowLevelRobotCommand& lhs, const rtt::LowLevelRobotCommand& rhs) {
    return
        lhs.id == rhs.id &&
        lhs.robot_vel == rhs.robot_vel &&
        lhs.ang == rhs.ang &&
        lhs.rot_cclockwise == rhs.rot_cclockwise &&
        lhs.w == rhs.w &&
        lhs.punt_power == rhs.punt_power &&
        lhs.do_kick == rhs.do_kick &&
        lhs.do_chip == rhs.do_chip &&
        lhs.forced == rhs.forced &&
        lhs.dribble_cclockwise == rhs.dribble_cclockwise &&
        lhs.dribble_vel == rhs.dribble_vel;
}

bool operator!=(const rtt::LowLevelRobotCommand& lhs, const rtt::LowLevelRobotCommand& rhs) {
    return !(lhs == rhs);
}
