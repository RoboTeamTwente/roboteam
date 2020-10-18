#ifndef __ROBOT_COMMAND_H
#define __ROBOT_COMMAND_H

#include <stdbool.h>
#include <stdint.h>

#include "BaseTypes.h"

typedef struct _RobotCommandPayload {
    uint8_t payload[PACKET_SIZE_ROBOT_COMMAND];
} RobotCommandPayload;

// Conversion constants
#define CONVERT_RHO 			0.004f
#define CONVERT_THETA 			0.00307f
#define CONVERT_YAW_REF 		0.00614f
#define CONVERT_SHOOTING_POWER 	0.39f
#define CONVERT_DRIBBLE_SPEED 	3.125f
#define CONVERT_VISION_YAW 		0.00307f

                                //Description                 Values            Units     	  Represented values    Bits
typedef struct _RobotCommand{
    PACKET_TYPE header;         //Packet type (RobotCommand)  -                 -             -                        8
    uint8_t id:8;               //Robot ID                    [0, 15]           -             [0, 15]                  4
    uint16_t rho:11;            //Velocity magnitude          [0, 2047]         0.004m/s      [0, 8.188]              11
    int16_t theta:11;           //Velocity angle              [-1024, 1023]     0.00307rad    [-pi, pi>               11
    int16_t angularVelocity:10; //Reference angular velocity  [-512, 511]       0.098rad/s    [-8*2pi, 8*2pi]         10
    uint8_t power:8;            //Kick/chip power             [0, 255]          0.39%         [0, 100]%                8
    bool doKick:1;              //Kick                        [0, 1]            -             {true, false}            1
    bool doChip:1;              //Chip                        [0, 1]            -             {true, false}            1
    bool force:1;               //Kick/chip immediately       [0, 1]            -             {true, false}            1
    bool debugInfo:1;           //Debug information           [0, 1]            -             {true, false}            1
    bool useCamInfo:1;          //Use camera information      [0, 1]            -             {true, false}            1
    uint8_t geneva:3;           //Geneva drive state          [0, 7]            -             [-2, 2]                  3
    uint8_t dribblerSpeed:5;    //Reference dribbler speed    [0, 31]          3.125%         [0, 100]%                5
    int16_t camRotation:11;     //Orientation (seen bycamera) [-1024, 1023]     0.00307rad    [-pi, pi]               11
} RobotCommand;

// Packet structure
// [---0--] [---1--] [---2--] [---3--] [---4--] [---5--] [---6--] [---7--] [---8--] [---9--]
// 11111111 -------- -------- -------- -------- -------- -------- -------- -------- -------- // header
// -------- 11111111 -------- -------- -------- -------- -------- -------- -------- -------- // id
// -------- -------- 11111111 111----- -------- -------- -------- -------- -------- -------- // rho
// -------- -------- -------- ---11111 111111-- -------- -------- -------- -------- -------- // theta
// -------- -------- -------- -------- ------11 11111111 -------- -------- -------- -------- // angularVelocity
// -------- -------- -------- -------- -------- -------- 11111111 -------- -------- -------- // power
// -------- -------- -------- -------- -------- -------- -------- 1------- -------- -------- // Kick
// -------- -------- -------- -------- -------- -------- -------- -1------ -------- -------- // Chip
// -------- -------- -------- -------- -------- -------- -------- --1----- -------- -------- // Force
// -------- -------- -------- -------- -------- -------- -------- ---1---- -------- -------- // debug info
// -------- -------- -------- -------- -------- -------- -------- ----1--- -------- -------- // use cam info
// -------- -------- -------- -------- -------- -------- -------- -----111 -------- -------- // geneva
// -------- -------- -------- -------- -------- -------- -------- -------- 11111--- -------- // dribbler speed
// -------- -------- -------- -------- -------- -------- -------- -------- -----111 11111111 // cam rotation


// -------------------------------------- GETTERS --------------------------------------
static inline PACKET_TYPE RobotCommand_getHeader(RobotCommandPayload *rcp){
    return (PACKET_TYPE)rcp->payload[0];
}
static inline uint8_t RobotCommand_getId(RobotCommandPayload *rcp){
    return rcp->payload[1];
}
static inline uint16_t RobotCommand_getRho(RobotCommandPayload *rcp){
    return ((rcp->payload[2] & 0b11111111) << 3)
         | ((rcp->payload[3] & 0b11100000) >> 5);
}
static inline int16_t RobotCommand_getTheta(RobotCommandPayload *rcp){
    return ((rcp->payload[3] & 0b00011111) << 6)
         | ((rcp->payload[4] & 0b11111100) >> 2);
}
static inline int16_t RobotCommand_getAngularVelocity(RobotCommandPayload *rcp){
    return ((rcp->payload[4] & 0b00000011) << 8)
         | ((rcp->payload[5] & 0b11111111) >> 0);
}
static inline uint8_t RobotCommand_getPower(RobotCommandPayload *rcp){
    return rcp->payload[6];
}
static inline bool RobotCommand_getDoKick(RobotCommandPayload *rcp){
    return rcp->payload[7] & 0b10000000 ? true : false;
}
static inline bool RobotCommand_getDoChip(RobotCommandPayload *rcp){
    return rcp->payload[7] & 0b01000000 ? true : false;
}
static inline bool RobotCommand_getForce(RobotCommandPayload *rcp){
    return rcp->payload[7] & 0b00100000 ? true : false;
}
static inline bool RobotCommand_getDebugInfo(RobotCommandPayload *rcp){
    return rcp->payload[7] & 0b00010000 ? true : false;
}
static inline bool RobotCommand_getUseCamInfo(RobotCommandPayload *rcp){
    return rcp->payload[7] & 0b00001000 ? true : false;
}
static inline uint8_t RobotCommand_getGeneva(RobotCommandPayload *rcp){
    return rcp->payload[7] & 0b00000111;
}
static inline uint8_t RobotCommand_getDribblerSpeed(RobotCommandPayload *rcp){
    return (rcp->payload[8] & 0b11111000) >> 3;
}
static inline int16_t RobotCommand_getCamRotation(RobotCommandPayload *rcp){
    return ((rcp->payload[8] & 0b00000111) << 8)
         | ((rcp->payload[9] & 0b11111111) >> 0);
}

// -------------------------------------- SETTERS --------------------------------------
// Note: functions assume an empty {0} packet to write to
static inline void RobotCommand_setHeader(RobotCommandPayload *rcp, PACKET_TYPE header){
    rcp->payload[0] = header;
}
static inline void RobotCommand_setId(RobotCommandPayload *rcp, uint8_t id){
    rcp->payload[1] = id;
}
static inline void RobotCommand_setRho(RobotCommandPayload *rcp, uint16_t rho){
    rcp->payload[2]  = (rho >> 3) & 0b11111111;
    rcp->payload[3] |= (rho << 5) & 0b11100000;
}
static inline void RobotCommand_setTheta(RobotCommandPayload *rcp, int16_t theta){
    rcp->payload[3] |= (theta >> 6) & 0b00011111;
    rcp->payload[4] |= (theta << 2) & 0b11111100;
}
static inline void RobotCommand_setAngularVelocity(RobotCommandPayload *rcp, int16_t angularVelocity){
    rcp->payload[4] |= (angularVelocity >> 8) & 0b00000011;
    rcp->payload[5] |= (angularVelocity >> 0) & 0b11111111;
}
static inline void RobotCommand_setPower(RobotCommandPayload *rcp, uint8_t power){
    rcp->payload[6]  = power;
}
static inline void RobotCommand_setDoKick(RobotCommandPayload *rcp, bool kick){
    rcp->payload[7] |= (kick << 7) & 0b10000000;
}
static inline void RobotCommand_setDoChip(RobotCommandPayload *rcp, bool chip){
    rcp->payload[7] |= (chip << 6) & 0b01000000;
}
static inline void RobotCommand_setForce(RobotCommandPayload *rcp, bool force){
    rcp->payload[7] |= (force << 5) & 0b00100000;
}
static inline void RobotCommand_setDebugInfo(RobotCommandPayload *rcp, bool debugInfo){
    rcp->payload[7] |= (debugInfo << 4) & 0b00010000;
}
static inline void RobotCommand_setUseCamInfo(RobotCommandPayload *rcp, bool useCamInfo){
    rcp->payload[7] |= (useCamInfo << 3) & 0b00001000;
}
static inline void RobotCommand_setGeneva(RobotCommandPayload *rcp, uint8_t geneva){
    rcp->payload[7] |= (geneva) & 0b00000111;
}
static inline void RobotCommand_setDribblerSpeed(RobotCommandPayload *rcp, uint8_t dribblerSpeed){
    rcp->payload[8] |= (dribblerSpeed << 3) & 0b11111000;
}
static inline void RobotCommand_setCamRotation(RobotCommandPayload *rcp, int16_t camRotation){
    rcp->payload[8] |= (camRotation << 8) & 0b00000111;
    rcp->payload[9]  = (camRotation << 0) & 0b11111111;
}

// -------------------------------------- ENCODE/DECODE --------------------------------------
static inline void decodeRobotCommand(RobotCommand *command, RobotCommandPayload *rcp){
    command->header = RobotCommand_getHeader(rcp);
    command->id = RobotCommand_getId(rcp);
    command->rho = RobotCommand_getRho(rcp);
    command->theta = RobotCommand_getTheta(rcp);
    command->angularVelocity = RobotCommand_getAngularVelocity(rcp);
    command->power = RobotCommand_getPower(rcp);
    command->doKick = RobotCommand_getDoKick(rcp);
    command->doChip = RobotCommand_getDoChip(rcp);
    command->force = RobotCommand_getForce(rcp);
    command->debugInfo = RobotCommand_getDebugInfo(rcp);
    command->useCamInfo = RobotCommand_getUseCamInfo(rcp);
    command->geneva = RobotCommand_getGeneva(rcp);
    command->dribblerSpeed = RobotCommand_getDribblerSpeed(rcp);
    command->camRotation = RobotCommand_getCamRotation(rcp);
}

static inline void encodeRobotCommand(RobotCommandPayload *rcp, RobotCommand *command){
    RobotCommand_setHeader(rcp, command->header);
    RobotCommand_setId(rcp, command->id);
    RobotCommand_setRho(rcp, command->rho);
    RobotCommand_setTheta(rcp, command->theta);
    RobotCommand_setAngularVelocity(rcp, command->angularVelocity);
    RobotCommand_setPower(rcp, command->power);
    RobotCommand_setDoKick(rcp, command->doKick);
    RobotCommand_setDoChip(rcp, command->doChip);
    RobotCommand_setForce(rcp, command->force);
    RobotCommand_setDebugInfo(rcp, command->debugInfo);
    RobotCommand_setUseCamInfo(rcp, command->useCamInfo);
    RobotCommand_setGeneva(rcp, command->geneva);
    RobotCommand_setDribblerSpeed(rcp, command->dribblerSpeed);
    RobotCommand_setCamRotation(rcp, command->camRotation);
}

#endif /*__ROBOT_COMMAND_H*/