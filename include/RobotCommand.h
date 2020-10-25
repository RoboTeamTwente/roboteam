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

<<<<<<< HEAD
/** ================================ PACKET ================================
[---0--] [---1--] [---2--] [---3--] [---4--] [---5--] [---6--] [---7--] [---8--] [---9--] [--10--]
11111111 -------- -------- -------- -------- -------- -------- -------- -------- -------- -------- header
-------- 1111---- -------- -------- -------- -------- -------- -------- -------- -------- -------- id
-------- ----1--- -------- -------- -------- -------- -------- -------- -------- -------- -------- doKick
-------- -----1-- -------- -------- -------- -------- -------- -------- -------- -------- -------- doChip
-------- ------1- -------- -------- -------- -------- -------- -------- -------- -------- -------- doForce
-------- -------1 -------- -------- -------- -------- -------- -------- -------- -------- -------- useCameraAngle
-------- -------- 11111111 11111111 -------- -------- -------- -------- -------- -------- -------- rho
-------- -------- -------- -------- 11111111 11111111 -------- -------- -------- -------- -------- theta
-------- -------- -------- -------- -------- -------- 11111111 11111111 -------- -------- -------- angle
-------- -------- -------- -------- -------- -------- -------- -------- 11111111 11111111 -------- cameraAngle
-------- -------- -------- -------- -------- -------- -------- -------- -------- -------- 111----- dribbler
-------- -------- -------- -------- -------- -------- -------- -------- -------- -------- ---111-- kickChipPower
-------- -------- -------- -------- -------- -------- -------- -------- -------- -------- ------1- angularControl
-------- -------- -------- -------- -------- -------- -------- -------- -------- -------- -------1 feedback
**/



/** ================================ STRUCT ================================ */
typedef struct _RobotCommand {
	uint8_t header;              // Header indicating packet type
	uint8_t id;                  // Id of the robot
	bool doKick;                 // Do a kick if ballsensor
	bool doChip;                 // Do a chip if ballsensor
	bool doForce;                // Do regardless of ballsensor
	bool useCameraAngle;         // Use the info in 'cameraAngle'
	uint16_t rho;                // Direction of movement
	uint16_t theta;              // Magnitude of movement (speed)
	uint16_t angle;              // Absolute angle / angular velocity
	uint16_t cameraAngle;        // Angle of the robot as seen by camera
	uint8_t dribbler;            // Dribbler speed
	uint8_t kickChipPower;       // Power of the kick or chip
	bool angularControl;         // 0 = angular velocity, 1 = absolute angle
	bool feedback;               // Ignore the packet. Just send feedback
} RobotCommand;



/** ================================ GETTERS ================================ */
static inline uint8_t RobotCommand_getHeader(RobotCommandPayload *rcp){
	return ((rcp->payload[0]));
}
static inline uint8_t RobotCommand_getId(RobotCommandPayload *rcp){
	return ((rcp->payload[1] & 0b11110000) >> 4);
}
static inline bool RobotCommand_getDoKick(RobotCommandPayload *rcp){
	return ((rcp->payload[1] & 0b00001000) > 0);
}
static inline bool RobotCommand_getDoChip(RobotCommandPayload *rcp){
	return ((rcp->payload[1] & 0b00000100) > 0);
}
static inline bool RobotCommand_getDoForce(RobotCommandPayload *rcp){
	return ((rcp->payload[1] & 0b00000010) > 0);
}
static inline bool RobotCommand_getUseCameraAngle(RobotCommandPayload *rcp){
	return ((rcp->payload[1] & 0b00000001) > 0);
}
static inline uint16_t RobotCommand_getRho(RobotCommandPayload *rcp){
	return ((rcp->payload[2]) << 8) | ((rcp->payload[3]));
}
static inline uint16_t RobotCommand_getTheta(RobotCommandPayload *rcp){
	return ((rcp->payload[4]) << 8) | ((rcp->payload[5]));
}
static inline uint16_t RobotCommand_getAngle(RobotCommandPayload *rcp){
	return ((rcp->payload[6]) << 8) | ((rcp->payload[7]));
}
static inline uint16_t RobotCommand_getCameraAngle(RobotCommandPayload *rcp){
	return ((rcp->payload[8]) << 8) | ((rcp->payload[9]));
}
static inline uint8_t RobotCommand_getDribbler(RobotCommandPayload *rcp){
	return ((rcp->payload[10] & 0b11100000) >> 5);
}
static inline uint8_t RobotCommand_getKickChipPower(RobotCommandPayload *rcp){
	return ((rcp->payload[10] & 0b00011100) >> 2);
}
static inline bool RobotCommand_getAngularControl(RobotCommandPayload *rcp){
	return ((rcp->payload[10] & 0b00000010) > 0);
}
static inline bool RobotCommand_getFeedback(RobotCommandPayload *rcp){
	return ((rcp->payload[10] & 0b00000001) > 0);
}



/** ================================ SETTERS ================================ */
static inline void RobotCommand_setHeader(RobotCommandPayload *rcp, uint8_t header){
	rcp->payload[0] = header;
=======
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
>>>>>>> cacbe83ecd21cd0040f90252cc00a33dedd047f5
}
static inline void RobotCommand_setId(RobotCommandPayload *rcp, uint8_t id){
	rcp->payload[1] = ((id << 4) & 0b11110000) | (rcp->payload[1] & 0b00001111);
}
static inline void RobotCommand_setDoKick(RobotCommandPayload *rcp, bool doKick){
	rcp->payload[1] = ((doKick << 3) & 0b00001000) | (rcp->payload[1] & 0b11110111);
}
static inline void RobotCommand_setDoChip(RobotCommandPayload *rcp, bool doChip){
	rcp->payload[1] = ((doChip << 2) & 0b00000100) | (rcp->payload[1] & 0b11111011);
}
static inline void RobotCommand_setDoForce(RobotCommandPayload *rcp, bool doForce){
	rcp->payload[1] = ((doForce << 1) & 0b00000010) | (rcp->payload[1] & 0b11111101);
}
static inline void RobotCommand_setUseCameraAngle(RobotCommandPayload *rcp, bool useCameraAngle){
	rcp->payload[1] = (useCameraAngle & 0b00000001) | (rcp->payload[1] & 0b11111110);
}
static inline void RobotCommand_setRho(RobotCommandPayload *rcp, uint16_t rho){
	rcp->payload[2] = (rho >> 8);
	rcp->payload[3] = rho;
}
static inline void RobotCommand_setTheta(RobotCommandPayload *rcp, uint16_t theta){
	rcp->payload[4] = (theta >> 8);
	rcp->payload[5] = theta;
}
static inline void RobotCommand_setAngle(RobotCommandPayload *rcp, uint16_t angle){
	rcp->payload[6] = (angle >> 8);
	rcp->payload[7] = angle;
}
static inline void RobotCommand_setCameraAngle(RobotCommandPayload *rcp, uint16_t cameraAngle){
	rcp->payload[8] = (cameraAngle >> 8);
	rcp->payload[9] = cameraAngle;
}
static inline void RobotCommand_setDribbler(RobotCommandPayload *rcp, uint8_t dribbler){
	rcp->payload[10] = ((dribbler << 5) & 0b11100000) | (rcp->payload[10] & 0b00011111);
}
static inline void RobotCommand_setKickChipPower(RobotCommandPayload *rcp, uint8_t kickChipPower){
	rcp->payload[10] = ((kickChipPower << 2) & 0b00011100) | (rcp->payload[10] & 0b11100011);
}
static inline void RobotCommand_setAngularControl(RobotCommandPayload *rcp, bool angularControl){
	rcp->payload[10] = ((angularControl << 1) & 0b00000010) | (rcp->payload[10] & 0b11111101);
}
static inline void RobotCommand_setFeedback(RobotCommandPayload *rcp, bool feedback){
	rcp->payload[10] = (feedback & 0b00000001) | (rcp->payload[10] & 0b11111110);
}



/** ================================ ENCODE ================================ */
static inline void decodeRobotCommand(RobotCommand *rc, RobotCommandPayload *rcp){
	rc->header              = RobotCommand_getHeader(rcp);
	rc->id                  = RobotCommand_getId(rcp);
	rc->doKick              = RobotCommand_getDoKick(rcp);
	rc->doChip              = RobotCommand_getDoChip(rcp);
	rc->doForce             = RobotCommand_getDoForce(rcp);
	rc->useCameraAngle      = RobotCommand_getUseCameraAngle(rcp);
	rc->rho                 = RobotCommand_getRho(rcp);
	rc->theta               = RobotCommand_getTheta(rcp);
	rc->angle               = RobotCommand_getAngle(rcp);
	rc->cameraAngle         = RobotCommand_getCameraAngle(rcp);
	rc->dribbler            = RobotCommand_getDribbler(rcp);
	rc->kickChipPower       = RobotCommand_getKickChipPower(rcp);
	rc->angularControl      = RobotCommand_getAngularControl(rcp);
	rc->feedback            = RobotCommand_getFeedback(rcp);
}



/** ================================ DECODE ================================ */
static inline void encodeRobotCommand(RobotCommandPayload *rcp, RobotCommand *rc){
	RobotCommand_setHeader(rcp, rc->header);
	RobotCommand_setId(rcp, rc->id);
	RobotCommand_setDoKick(rcp, rc->doKick);
	RobotCommand_setDoChip(rcp, rc->doChip);
	RobotCommand_setDoForce(rcp, rc->doForce);
	RobotCommand_setUseCameraAngle(rcp, rc->useCameraAngle);
	RobotCommand_setRho(rcp, rc->rho);
	RobotCommand_setTheta(rcp, rc->theta);
	RobotCommand_setAngle(rcp, rc->angle);
	RobotCommand_setCameraAngle(rcp, rc->cameraAngle);
	RobotCommand_setDribbler(rcp, rc->dribbler);
	RobotCommand_setKickChipPower(rcp, rc->kickChipPower);
	RobotCommand_setAngularControl(rcp, rc->angularControl);
	RobotCommand_setFeedback(rcp, rc->feedback);
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