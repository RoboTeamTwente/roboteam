#include "roboteam_robothub/packing.h"

#include <array>
#include <cassert>
#include <cstdint>
#include <iostream>
#include <memory>
#include <boost/optional.hpp>
#include <math.h>

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
 *                      Gyroscope info
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
    LowLevelRobotCommand createLowLevelRobotCommand(roboteam_msgs::RobotCommand const &command,
                                                    b::optional<roboteam_msgs::World> const &worldOpt) {

        using roboteam_msgs::RobotCommand;



        // Calculate robot angle
        double rawAng = velocityVec.angle();




        /*
        Normalize to [-180,180):
        double constrainAngle(double x){
            x = fmod(x + 180,360);
            if (x < 0)
                x += 360;
            return x - 180;
        }*/

        float velocity_angular = command.w;

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
            auto const &world = *worldOpt;
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


//        llcommand.ang = ang;
//        llcommand.rot_cclockwise = rot_cclockwise;
//        llcommand.w = w;
//        llcommand.punt_power = std::max(kick_force, chip_force);
//        llcommand.do_kick = do_kick;
//        llcommand.do_chip = do_chip;
//        llcommand.forced = do_forced;
//        llcommand.dribble_cclockwise = dribble_cclockwise;
//        llcommand.dribble_vel = dribble_vel;
//
//        llcommand.cam_data_on = cam_data_on;
//        llcommand.cam_robot_vel = cam_robot_vel;
//        llcommand.cam_ang = cam_ang;
//        llcommand.cam_w = cam_w;

        LowLevelRobotCommand llrc;

        llrc.id                 = command.id;
        llrc.velocity_x         = command.x_vel * 128;
        llrc.velocity_y         = command.y_vel * 128;
        llrc.driving_reference  = false;
        llrc.use_cam_info       = false;
        llrc.rotation_direction = command.w > 0;
        llrc.velocity_angular   = command.w;
        llrc.debug_info         = false;
        llrc.do_kick            = command.kicker;
        llrc.do_chip            = command.chipper;
        llrc.kick_chip_forced   = command.kicker_forced || command.chipper_forced;
        llrc.kick_chip_power    = command.kicker ? command.kicker_vel : command.chipper_vel;
        llrc.velocity_dribbler  = command.dribbler;
        llrc.geneva_drive_state = command.geneva_state;
        llrc.cam_position_x     = 0;
        llrc.cam_position_y     = 0;
        llrc.cam_rotation       = 0;

        return llrc;
    }

    boost::optional<packed_protocol_message> createRobotPacket(
        roboteam_msgs::RobotCommand const &command, b::optional<roboteam_msgs::World> const &world){
        auto llcommand = createLowLevelRobotCommand(command, world);
        return createRobotPacket(llcommand);
    }

    bool inRange(int val, int min, int max){
        return (min <= val && val <= max);
    }

    /* As described in this comment https://roboteamtwente2.slack.com/files/U6CPQLJ6S/F9V330Z2N/packet_proposal.txt */
    boost::optional<packed_protocol_message> createRobotPacket(LowLevelRobotCommand llrc){

        // Holds if all the values of the command are in range
        bool valuesInRange = true;

        // inRange(val, min, max) is inclusive!;
        // inRange(-5, -5, 10) -> true; // inRange(14, -5, 10) -> false;
        // inRange(10, -5, 10) -> true; // inRange(-8, -5, 10) -> false;

        valuesInRange &= inRange(llrc.id, 0, 15);
        valuesInRange &= inRange(llrc.velocity_x, -1023, 1024);
        valuesInRange &= inRange(llrc.velocity_y, -1023, 1024);
        valuesInRange &= inRange(llrc.velocity_angular, 0, 511);
        valuesInRange &= inRange(llrc.kick_chip_power, 0, 255);
        valuesInRange &= inRange(llrc.velocity_dribbler, 0, 255);
        valuesInRange &= inRange(llrc.geneva_drive_state, 0, 7);
        valuesInRange &= inRange(llrc.cam_position_x, -4096, 4095);
        valuesInRange &= inRange(llrc.cam_position_y, -4096, 4095);
        valuesInRange &= inRange(llrc.cam_rotation, -1024, 1023);

        // Values are automatically limited in the code below, but returning boost::none is a good idea nonetheless.
        if(!valuesInRange){
            return boost::none;
        }

        //Might still be wrong because certain people can't make their minds up
        packed_protocol_message byteArr;

        // Static cast truncates to the last 8 bits
        // is implementation defined though.

        byteArr[0] = static_cast<uint8_t>(  // aaaaabbb
            (0b11111000 & (llrc.id << 3)) |                 //aaaaa000   5 bits; bits  4-0 to 7-3
            (0b00000111 & (llrc.velocity_x >> 8))           //00000bbb  11 bits; bits 10-8 to 2-0
        );

        byteArr[1] = static_cast<uint8_t>(  // bbbbbbbb
            llrc.velocity_x                                 //bbbbbbbb  11 bits; bits  7-0 to 7-0
        );

        byteArr[2] = static_cast<uint8_t>(  // cccccccc
            llrc.velocity_y >> 3                            // cccccccc 11 bits; bits 10-8 to 7-0
        );

        byteArr[3] = static_cast<uint8_t>(  // cccde0fg
            (0b11100000 & (llrc.velocity_y << 5)) |         // ccc00000 11 bits; bits  2-0 to 7-5
            (0b00010000 & (llrc.driving_reference << 4)) |  // 000d0000  1 bit ; bit     0 to   4
            (0b00001000 & (llrc.use_cam_info) << 3) |       // 0000e000  1 bit ; bit     0 to   3
            (0b00000010 & (llrc.rotation_direction << 1)) | // 000000f0  1 bit ; bit     0 to   1
            (0b00000001 & (llrc.velocity_angular >> 8))     // 0000000g  9 bits; bit     8 to   0
        );

        byteArr[4] = static_cast<uint8_t>(  // gggggggg
            llrc.velocity_angular                           // gggggggg  8 bits; bits  7-0 to 7-0
        );

        byteArr[5] = static_cast<uint8_t>(  // 0000hijk
            (0b00001000 & (llrc.debug_info << 3)) |         // 0000h000  1 bit ; bit     0 to   3
            (0b00000100 & (llrc.do_kick << 2)) |            // 00000i00  1 bit ; bit     0 to   2
            (0b00000010 & (llrc.do_chip << 1)) |            // 000000j0  1 bit ; bit     0 to   1
            (0b00000001 & (llrc.kick_chip_forced << 2))     // 0000000k  1 bit ; bit     0 to   0
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

    std::string byteToBinary(uint8_t const &byte) {
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

    OldACK decodeOldACK(packed_old_ack const &packedAck) {
        OldACK ack;

        // Decode ID
        if (packedAck[0] >= '0' && packedAck[0] <= '9') {
            ack.robotID = static_cast<int>(packedAck[0] - '0');
        } else if (packedAck[0] >= 'A' && packedAck[0] <= 'F') {
            ack.robotID = static_cast<int>(packedAck[0] - 'A');
        } else {
            ack.robotID = -1;
        }

        // Decode robot ACK
        ack.robotACK = packedAck[1] == '1';

        // Decode random value
        // ack.randomValue = static_cast<uint8_t>(packedAck[2]);

//    for(int i = 0; i < 2; i++) {
//        std::cout << "packedAck[" << i << "] : " << byteToBinary(packedAck[i]) << "\n";
//    }
        return ack;
    }

    NewACK decodeNewACK(packed_new_ack const &packedAck) {
        NewACK ack;

        // Decode ID
        if (packedAck[0] >= '0' && packedAck[0] <= '9') {
            ack.robotID = static_cast<int>(packedAck[0] - '0');
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
        };

        auto decodeWheelSpeed = [](uint8_t wheelInfo) {
            return static_cast<int>(wheelInfo & 127);
        };

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

bool operator==(const rtt::LowLevelRobotCommand &lhs, const rtt::LowLevelRobotCommand &rhs) {
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

bool operator!=(const rtt::LowLevelRobotCommand &lhs, const rtt::LowLevelRobotCommand &rhs) {
    return !(lhs == rhs);
}
