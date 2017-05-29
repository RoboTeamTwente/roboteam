#include "roboteam_robothub/packing.h"

#include <array>
#include <cassert>
#include <cstdint>
#include <iostream>
#include <memory>
#include <boost/optional.hpp>
namespace b = boost;

#include "roboteam_msgs/RobotCommand.h"
#include "roboteam_msgs/World.h"
#include "roboteam_utils/Vector2.h"
#include "roboteam_utils/Math.h"

namespace {

}

namespace rtt {

/**
 * Command packet format, inspired by the (old?) RoboJackets protocol.
 *
 * Note: l and o are skipped because they are hard to discern from badly written 0's and 1's.
 *
 * Byte     Config      Description
 * 0        aaaabbbb    aaaa: Robot ID, bbbb: Robot velocity
 * 1        bbbbbbbb    bbbbbbbb: Robot velocity, 0 - 8191 (mm/s)
 * 2        bccccccc    b: Robot velocity, cccccccc: Moving direction, resolution (2 * pi / 512) (rad)
 * 3        cc00deee    cc: Moving direction, d: Rotation direction, true is counter clockwise
 * 4        eeeeeeee    eeeeeeeeeee: Angular velocity, 0 - 2047 (deg/s)
 * 5        ffffffff    ffffffff: Kick force, 0 - 255
 * 6        rghijkkk    r: whether or not the 8th, 9th, 10th, 11th, and 12th byte are meaningful
 *                      g: whether or not to kick
 *                      h: whether or not to chip
 *                      i: forced or not
 *                      j: counterclockwise dribbler. Counterclockwise means spinning in a way s.t. the ball does not spin toward the robot (i.e. with the robot facing towards the right).
 *                      kkk: dribbler speed, 0 - 7
 * 7        nnnnnnnn    nnnnnnnn: robot velocity as perceived by camera. 0 - 8191 (mm/s)
 * 8        nnnnnppp    
 * 9        ppppppqq    ppppppppp: Moving direction as perceived by camera, resolution (2 * pi / 512) (rad) 
 * 10       qqqqqqqq    qqqqqqqqqqq: Angular velocity as perceived by the camera, 0 - 2047 (deg/s)
 * 11       qm000000    m: true if rotation direction as perceived by camera is counterclockwise. counterclockwise as seen by the camera from above
 */

/**
 *  Old packet response format
 *
 *  Byte    Config      Description
 *  0       aaaaaaaa    Robot ID (ASCII, hexadecimal!)
 *  1       bbbbbbbb    ACK or NACK byte. Either '0' or '1'. (ASCII, hexadecimal)
 */

/**
 * New packet response format, inspired by Hans, Jim & Bob.
 *
 * The first few things (id, ack/nack, battery critical, possibly ball) can possibly be compressed in the first byte.
 * Maybe we can also add a bit to the command packet that turns these things on or off (lean mode)?
 *
 * l and o are skipped because they are hard to discern from 0 and 1 in longhand
 *
 * ASCII size:      7 bytes
 * Compressed size: 5 bytes
 * Lean size:       1 byte (format yet to be determined)
 *
 * Byte     Config      Description
 * 0        aaaaaaaa    a: Robot ID (ASCII, hexadecimal!)
 * 1        bbbbbbbb    b: ACK or NACK byte (whether or not the command was actually
 *                         dispatched from the base station) (ASCII)
 * 2        cccccccc    c: Battery critical indicator (ASCII)
 * 3        dddddddd    d: Ball sensor (0 - 255, or 0-1) (can either be where it sees it, or how sure it is) (ASCII)
 *
 *                      Wheels
 *                          Wheel speed is 0 - 127, rounded to the nearest integer (rad / s)
 *                          Wheel direction: 0 is counterclockwise, 1 is clockwise
 * 4        efffffff    Front left. (wheel 1)
 *                      e: wheel direction 
 *                      f: wheel speed
 * 5        ghhhhhhh    Front right (wheel 4)
 *                      g: wheel direction
 *                      h: wheel speed
 * 6        ijjjjjjj    Back left (wheel 2)
 *                      i: wheel direction
 *                      j: wheel speed
 * 7        kmmmmmmm    Back right (wheel 3)
 *                      k: wheel direction
 *                      m: wheel speed
 *
 * Optional additions for later:
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

// Copy of getWorldBot() because I don't want to pull in tactics as a dependency. If this function is moved to utils
// we can use that
boost::optional<roboteam_msgs::WorldRobot> getWorldBot(unsigned int id, bool ourTeam, const roboteam_msgs::World& world) {
    std::vector<roboteam_msgs::WorldRobot> bots = ourTeam ? world.us : world.them;
    for (const auto& bot : bots) {
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
LowLevelRobotCommand createLowLevelRobotCommand(roboteam_msgs::RobotCommand const & command, b::optional<roboteam_msgs::World> const & worldOpt) {
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

    if (w > PACKET_MAX_W) w = PACKET_MAX_W;
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

    /////////////////////////
    // Figure out cam data //
    /////////////////////////
    
    bool cam_data_on = false;
    int32_t cam_robot_vel = 0;
    int32_t cam_ang = 0;
    bool cam_rot_cclockwise = 0;
    int32_t cam_w = 0;

    if (worldOpt) {
        auto const & world = *worldOpt;
        auto botOpt = getWorldBot(command.id, true, world);

        if (botOpt) {
            auto bot = *botOpt;

            cam_data_on = true;

            Vector2 camVelocityVec(bot.vel.x, bot.vel.y);
            cam_robot_vel = camVelocityVec.length();

            double ang = cleanAngle(camVelocityVec.angle() - bot.angle);
            
            if (ang < 0) {
                ang += 2 * M_PI;
            }

            cam_ang = ang / (2 * M_PI / 512.0);
            
            if (bot.w > 0) {
                cam_rot_cclockwise = true;
                cam_w = bot.w;
            } else if (bot.w < 0) {
                cam_rot_cclockwise = false;
                cam_w = -bot.w;
            } 
        }
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

    llcommand.cam_data_on = cam_data_on;
    llcommand.cam_robot_vel = cam_robot_vel;
    llcommand.cam_ang = cam_ang;
    llcommand.cam_w = cam_w;

    // std::cout << "[RobotHub] ------------------------------------------\n";
    // std::cout << "[RobotHub] About to send the following w to the motor: " << llcommand.w << "\n";
    // std::cout << "[RobotHub] rot_cclockwise: " << (int) llcommand.rot_cclockwise << "\n";

    return llcommand;
}

/**
 * TODO: Measure our highest kicking power and use that to scale kick_vel!
 */
boost::optional<packed_protocol_message> createRobotPacket( roboteam_msgs::RobotCommand const & command
                                                          , b::optional<roboteam_msgs::World> const & world) {
    auto llcommand = createLowLevelRobotCommand(command, world);

    return createRobotPacket(
            llcommand.id,
            llcommand.robot_vel,
            llcommand.ang,
            llcommand.rot_cclockwise,
            llcommand.w,
            llcommand.punt_power,
            llcommand.cam_data_on,
            llcommand.do_kick,
            llcommand.do_chip,
            llcommand.forced,
            llcommand.dribble_cclockwise,
            llcommand.dribble_vel,
            llcommand.cam_robot_vel,
            llcommand.cam_ang,
            llcommand.cam_rot_cclockwise,
            llcommand.cam_w
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
            llrc.cam_data_on,
            llrc.do_kick,
            llrc.do_chip,
            llrc.forced,
            llrc.dribble_cclockwise,
            llrc.dribble_vel,
            llrc.cam_robot_vel,
            llrc.cam_ang,
            llrc.cam_rot_cclockwise,
            llrc.cam_w
            );
}

boost::optional<packed_protocol_message> createRobotPacket(int32_t id, int32_t robot_vel, int32_t ang,
                                                         bool rot_cclockwise, int32_t w, uint8_t punt_power,
                                                         bool do_kick, bool do_chip, bool forced,
                                                         bool dribble_cclockwise, uint8_t dribble_vel) {
    return createRobotPacket(
            id,
            robot_vel,
            ang,
            rot_cclockwise,
            w,
            punt_power,
            false,
            do_kick,
            do_chip,
            forced,
            dribble_cclockwise,
            dribble_vel,
            0,
            0,
            false,
            0
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
                                                         bool cam_data_on,
                                                         bool do_kick, bool do_chip, bool forced,
                                                         bool dribble_cclockwise, uint8_t dribble_vel,
                                                         int32_t cam_robot_vel, int32_t cam_ang,
                                                         bool cam_rot_cclockwise, int32_t cam_w) {
    if (!((id >= 0 && id < 16)
            && (robot_vel >= 0 && robot_vel < 8192)
            && (ang >= 0 && ang < 512)
            && (w >= 0 && w < 2048)
            && (dribble_vel >= 0 && dribble_vel < 8))
            && (cam_robot_vel >= 0 && cam_robot_vel < 8192)
            && (cam_ang >= 0 && cam_ang < 512)
            && (cam_w >= 0 && cam_w < 2048)) {
        return boost::none;
    }

    packed_protocol_message byteArr;

    // Static cast truncates to the last 8 bits
    // is implementation defined though.

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
    // First the cam data on bit, then
    // the chip and forced bools, then a bool that designates
    // a clockwise dribbler, and then three bits to designate dribble velocity
    byteArr[6] = static_cast<uint8_t>(cam_data_on << 7)
                    | static_cast<uint8_t>(do_kick << 6)
                    | static_cast<uint8_t>(do_chip << 5)
                    | static_cast<uint8_t>(forced << 4)
                    | static_cast<uint8_t>(dribble_cclockwise << 3)
                    | static_cast<uint8_t>(dribble_vel & 7);
    
    byteArr[7] = static_cast<uint8_t>(cam_robot_vel >> 5);

    byteArr[8] = static_cast<uint8_t>(cam_robot_vel << 3)
                | static_cast<uint8_t>(cam_ang >> 6);

    byteArr[9] = static_cast<uint8_t>(cam_ang << 2)
                | static_cast<uint8_t>(cam_w >> 9);

    byteArr[10] = static_cast<uint8_t>(cam_w >> 1);

    byteArr[11] = static_cast<uint8_t>((cam_w & 1) << 7)
                | static_cast<uint8_t>(cam_rot_cclockwise << 6);

    return boost::optional<packed_protocol_message>(byteArr);
}

std::string byteToBinary(uint8_t const & byte) {
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

OldACK decodeOldACK(packed_old_ack const & packedAck) {
    OldACK ack;

    // Decode ID
    if (packedAck[0] >= '0' && packedAck[0] <= '9') {
        ack.robotID = static_cast<int>(packedAck[0] - '9');
    } else if (packedAck[0] >= 'A' && packedAck[0] <= 'F') {
        ack.robotID = static_cast<int>(packedAck[0] - 'A');
    } else {
        ack.robotID = -1;
    }

    // Decode robot ACK
    ack.robotACK = packedAck[1] == '1';

    return ack;
}

NewACK decodeNewACK(packed_new_ack const & packedAck) {
    NewACK ack;

    // Decode ID
    if (packedAck[0] >= '0' && packedAck[0] <= '9') {
        ack.robotID = static_cast<int>(packedAck[0] - '9');
    } else if (packedAck[0] >= 'A' && packedAck[0] <= 'F') {
        ack.robotID = static_cast<int>(packedAck[0] - 'A');
    } else {
        ack.robotID = -1;
    }

    // Decode robot ACK info
    ack.robotACK = packedAck[1] == '1';

    ack.batteryCritical = packedAck[2] == '1';

    ack.ballSensor = static_cast<int>(packedAck[3]);

    // Decode wheel speeds
    auto decodeWheelDir = [](uint8_t wheelInfo) {
        return static_cast<bool>((wheelInfo >> 7) & 1);
    } ;
    
    auto decodeWheelSpeed = [](uint8_t wheelInfo) {
        return static_cast<int>(wheelInfo & 127);
    } ;

    ack.flWheelDir = decodeWheelDir(packedAck[4]);
    ack.flWheelSpeed = decodeWheelSpeed(packedAck[4]);

    ack.frWheelDir = decodeWheelDir(packedAck[5]);
    ack.frWheelSpeed = decodeWheelSpeed(packedAck[5]);

    ack.blWheelDir = decodeWheelDir(packedAck[6]);
    ack.blWheelSpeed = decodeWheelSpeed(packedAck[6]);

    ack.brWheelDir = decodeWheelDir(packedAck[7]);
    ack.brWheelSpeed = decodeWheelSpeed(packedAck[7]);
    
    return ack;
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
