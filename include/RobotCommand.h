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


#endif /*__ROBOT_COMMAND_H*/