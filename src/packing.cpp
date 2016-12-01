#include "roboteam_robothub/packing.h"

#include <array>
#include <cassert>
#include <cstdint>
#include <iostream>
#include <memory>
#include <boost/optional.hpp>

namespace rtt {

/**
 * Packet format, inspired by the (old?) RoboJackets protocol.
 *
 * Byte     Config      Description
 * 1        aaaabbbb    aaaa: Robot ID, bbbb: Robot velocity
 * 2        bbbbbbbb    bbbbbbbb: Robot velocity, 0 - 4095 (mm/s)
 * 3        cccccccc    cccccccc: Moving direction, resolution 2 * pi / 512
 * 4        000cdeee    c: Moving direction, d: Rotation direction,
 * 5        eeeeeeee    eeeeeeeeeee: Angular velocity, 0-2047 (deg/s)
 * 6        ffffffff    ffffffff: Kick force, 0 - 255
 * 7        0ghijkkk    g: whether or not to kick
 *                      h: whether or not to chip
 *                      i: forced or not
 *                      j: counterclockwise dribbler
 *                      kkk: dribbler speed, 0 - 7
 */

/**
 * All mentioned ranges are inclusive. If the specified ranges are violated,
 * the function returns boost::none.
 *
 * \param id Must be between 0 and 15
 * \param robot_vel Must be between 0 and 4095 (mm/s)
 * \param w Moving direction. Must be between 0 and 511. Resolution
 *          is 2*pi/512
 * \param rot_cclockwise True if rotation is counter clockwise
 * \param w_vel Angular velocity. Must be between 0 and 2047
 * \param kick_force Naturally between 0 and 255
 * \param do_kick Whether or not a chip or kick should be done
 * \param chip True if the robot should chip, otherwise a kick will be done
 * \param forced If true robot will not wait until it's close to the ball
 * \param dribble_cclockwise If true dribbler will spin counter clockwise
 * \param dribble_vel Must be between 0 and 7.
 */
boost::optional<packed_protocol_message> createRobotPacket(int id, int robot_vel, int w,
                                                         bool rot_cclockwise, int w_vel, uint8_t kick_force,
                                                         bool do_kick, bool chip, bool forced,
                                                         bool dribble_cclockwise, uint8_t dribble_vel) {
    if (!((id >= 0 && id < 16)
            && (robot_vel >= 0 && robot_vel < 4096)
            && (w >= 0 && w < 512)
            && (w_vel >= 0 && w_vel < 2048)
            && (dribble_vel >= 0 && dribble_vel < 8))) {
        return boost::none;
    }

    packed_protocol_message byteArr;

    // First nibble are the robot id
    // Second nibble are the third nibble of robot velocity
    byteArr[0] = static_cast<uint8_t>(((id & 15) << 4) | ((robot_vel >> 8) & 15));
    // First and second nibble of robot velocity
    byteArr[1] = static_cast<uint8_t>(robot_vel);
    // Second to ninth bit of moving direction
    byteArr[2] = static_cast<uint8_t>(w >> 1);
    // First bit of moving direction
    // Then bit that designates clockwise rotation or not
    // Last three bits of angular velocity
    byteArr[3] = static_cast<uint8_t>((w & 1) << 4) 
                | static_cast<uint8_t>(rot_cclockwise << 3)
                | static_cast<uint8_t>((w_vel >> 8) & 7);
    // First two nibbles of angular velocity
    byteArr[4] = static_cast<uint8_t>(w_vel);
    // Just plug in the byte of kick-force
    byteArr[5] = kick_force;
    // gggg = 0, 0, chip = 1 kick = 0, forced = 1 normal = 0
    // First the chip and forced bools, then a bool that designates
    // a clockwise dribbler, and then three bits to designate dribble velocity
    byteArr[6] = static_cast<uint8_t>(do_kick << 6)
                    | static_cast<uint8_t>(chip << 5)
                    | static_cast<uint8_t>(forced << 4)
                    | static_cast<uint8_t>(dribble_cclockwise << 3)
                    | static_cast<uint8_t>(dribble_vel & 7);

    return boost::optional<packed_protocol_message>(byteArr);
}

}
