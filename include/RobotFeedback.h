#ifndef __ROBOTFEEDBACK_H
#define __ROBOTFEEDBACK_H

#include <stdbool.h>
#include <stdint.h>

#include "BaseTypes.h"

typedef struct _RobotFeedbackPayload {
    uint8_t payload[PACKET_SIZE_ROBOT_FEEDBACK];
} RobotFeedbackPayload;

<<<<<<< HEAD
/** ================================ PACKET ================================
[---0--] [---1--] [---2--] [---3--] [---4--] [---5--] [---6--] [---7--] [---8--] [---9--] [--10--]
11111111 -------- -------- -------- -------- -------- -------- -------- -------- -------- -------- header
-------- 1111---- -------- -------- -------- -------- -------- -------- -------- -------- -------- id
-------- ----1111 -------- -------- -------- -------- -------- -------- -------- -------- -------- battery_level
-------- -------- 1------- -------- -------- -------- -------- -------- -------- -------- -------- xsens_calibrated
-------- -------- -1------ -------- -------- -------- -------- -------- -------- -------- -------- ballsensor_working
-------- -------- --1----- -------- -------- -------- -------- -------- -------- -------- -------- has_ball
-------- -------- ---1---- -------- -------- -------- -------- -------- -------- -------- -------- capacitor_charged
-------- -------- ----1111 -------- -------- -------- -------- -------- -------- -------- -------- ball_position
-------- -------- -------- 11111111 11111111 -------- -------- -------- -------- -------- -------- rho
-------- -------- -------- -------- -------- 11111111 11111111 -------- -------- -------- -------- theta
-------- -------- -------- -------- -------- -------- -------- 11111111 11111111 -------- -------- angle
-------- -------- -------- -------- -------- -------- -------- -------- -------- 1111---- -------- wheel_locked
-------- -------- -------- -------- -------- -------- -------- -------- -------- ----1111 -------- wheel_slipping
-------- -------- -------- -------- -------- -------- -------- -------- -------- -------- 1111---- rssi
**/



/** ================================ STRUCT ================================ */
typedef struct _RobotFeedback {
	uint8_t header;              // Header byte indicating the type of packet
	uint8_t id;                  // Id of the robot 
	uint8_t battery_level;       // The voltage level of the battery
	bool xsens_calibrated;       // Indicates if the XSens IMU is calibrated
	bool ballsensor_working;     // Indicates if the ballsensor is working
	bool has_ball;               // Indicates if the ball is somewhere in front of the ballsensor
	bool capacitor_charged;      // Indicates if the capacitor for kicking and chipping is charged
	uint8_t ball_position;       // Indicates where in front of the ballsensor the ball is
	uint16_t rho;                // The estimated direction of movement
	uint16_t theta;              // The estimated magnitude of movement (speed)
	uint16_t angle;              // The estimated angle
	uint8_t wheel_locked;        // Indicates if a wheel is locked. One bit per wheel
	uint8_t wheel_slipping;      // Indicates if a wheel is slipping. One bit per wheel
	uint8_t rssi;                // Signal strength of the last packet received by the robot
} RobotFeedback;



/** ================================ GETTERS ================================ */
static inline uint8_t RobotFeedback_getHeader(RobotFeedbackPayload *rfp){
	return ((rfp->payload[0]));
}
static inline uint8_t RobotFeedback_getId(RobotFeedbackPayload *rfp){
	return ((rfp->payload[1] & 0b11110000) >> 4);
}
static inline uint8_t RobotFeedback_getBattery_level(RobotFeedbackPayload *rfp){
	return ((rfp->payload[1] & 0b00001111));
}
static inline bool RobotFeedback_getXsens_calibrated(RobotFeedbackPayload *rfp){
	return ((rfp->payload[2] & 0b10000000) > 0);
}
static inline bool RobotFeedback_getBallsensor_working(RobotFeedbackPayload *rfp){
	return ((rfp->payload[2] & 0b01000000) > 0);
}
static inline bool RobotFeedback_getHas_ball(RobotFeedbackPayload *rfp){
	return ((rfp->payload[2] & 0b00100000) > 0);
}
static inline bool RobotFeedback_getCapacitor_charged(RobotFeedbackPayload *rfp){
	return ((rfp->payload[2] & 0b00010000) > 0);
}
static inline uint8_t RobotFeedback_getBall_position(RobotFeedbackPayload *rfp){
	return ((rfp->payload[2] & 0b00001111));
}
static inline uint16_t RobotFeedback_getRho(RobotFeedbackPayload *rfp){
	return ((rfp->payload[3]) << 8) | ((rfp->payload[4]));
}
static inline uint16_t RobotFeedback_getTheta(RobotFeedbackPayload *rfp){
	return ((rfp->payload[5]) << 8) | ((rfp->payload[6]));
}
static inline uint16_t RobotFeedback_getAngle(RobotFeedbackPayload *rfp){
	return ((rfp->payload[7]) << 8) | ((rfp->payload[8]));
}
static inline uint8_t RobotFeedback_getWheel_locked(RobotFeedbackPayload *rfp){
	return ((rfp->payload[9] & 0b11110000) >> 4);
}
static inline uint8_t RobotFeedback_getWheel_slipping(RobotFeedbackPayload *rfp){
	return ((rfp->payload[9] & 0b00001111));
}
static inline uint8_t RobotFeedback_getRssi(RobotFeedbackPayload *rfp){
	return ((rfp->payload[10] & 0b11110000) >> 4);
}



/** ================================ SETTERS ================================ */
static inline void RobotFeedback_setHeader(RobotFeedbackPayload *rfp, uint8_t header){
	rfp->payload[0] = header;
}
static inline void RobotFeedback_setId(RobotFeedbackPayload *rfp, uint8_t id){
	rfp->payload[1] = ((id << 4) & 0b11110000) | (rfp->payload[1] & 0b00001111);
}
static inline void RobotFeedback_setBattery_level(RobotFeedbackPayload *rfp, uint8_t battery_level){
	rfp->payload[1] = (battery_level & 0b00001111) | (rfp->payload[1] & 0b11110000);
}
static inline void RobotFeedback_setXsens_calibrated(RobotFeedbackPayload *rfp, bool xsens_calibrated){
	rfp->payload[2] = ((xsens_calibrated << 7) & 0b10000000) | (rfp->payload[2] & 0b01111111);
}
static inline void RobotFeedback_setBallsensor_working(RobotFeedbackPayload *rfp, bool ballsensor_working){
	rfp->payload[2] = ((ballsensor_working << 6) & 0b01000000) | (rfp->payload[2] & 0b10111111);
}
static inline void RobotFeedback_setHas_ball(RobotFeedbackPayload *rfp, bool has_ball){
	rfp->payload[2] = ((has_ball << 5) & 0b00100000) | (rfp->payload[2] & 0b11011111);
}
static inline void RobotFeedback_setCapacitor_charged(RobotFeedbackPayload *rfp, bool capacitor_charged){
	rfp->payload[2] = ((capacitor_charged << 4) & 0b00010000) | (rfp->payload[2] & 0b11101111);
}
static inline void RobotFeedback_setBall_position(RobotFeedbackPayload *rfp, uint8_t ball_position){
	rfp->payload[2] = (ball_position & 0b00001111) | (rfp->payload[2] & 0b11110000);
}
static inline void RobotFeedback_setRho(RobotFeedbackPayload *rfp, uint16_t rho){
	rfp->payload[3] = (rho >> 8);
	rfp->payload[4] = rho;
}
static inline void RobotFeedback_setTheta(RobotFeedbackPayload *rfp, uint16_t theta){
	rfp->payload[5] = (theta >> 8);
	rfp->payload[6] = theta;
}
static inline void RobotFeedback_setAngle(RobotFeedbackPayload *rfp, uint16_t angle){
	rfp->payload[7] = (angle >> 8);
	rfp->payload[8] = angle;
}
static inline void RobotFeedback_setWheel_locked(RobotFeedbackPayload *rfp, uint8_t wheel_locked){
	rfp->payload[9] = ((wheel_locked << 4) & 0b11110000) | (rfp->payload[9] & 0b00001111);
}
static inline void RobotFeedback_setWheel_slipping(RobotFeedbackPayload *rfp, uint8_t wheel_slipping){
	rfp->payload[9] = (wheel_slipping & 0b00001111) | (rfp->payload[9] & 0b11110000);
}
static inline void RobotFeedback_setRssi(RobotFeedbackPayload *rfp, uint8_t rssi){
	rfp->payload[10] = ((rssi << 4) & 0b11110000) | (rfp->payload[10] & 0b00001111);
}



/** ================================ ENCODE ================================ */
static inline void decodeRobotFeedback(RobotFeedback *rf, RobotFeedbackPayload *rfp){
	rf->header              = RobotFeedback_getHeader(rfp);
	rf->id                  = RobotFeedback_getId(rfp);
	rf->battery_level       = RobotFeedback_getBattery_level(rfp);
	rf->xsens_calibrated    = RobotFeedback_getXsens_calibrated(rfp);
	rf->ballsensor_working  = RobotFeedback_getBallsensor_working(rfp);
	rf->has_ball            = RobotFeedback_getHas_ball(rfp);
	rf->capacitor_charged   = RobotFeedback_getCapacitor_charged(rfp);
	rf->ball_position       = RobotFeedback_getBall_position(rfp);
	rf->rho                 = RobotFeedback_getRho(rfp);
	rf->theta               = RobotFeedback_getTheta(rfp);
	rf->angle               = RobotFeedback_getAngle(rfp);
	rf->wheel_locked        = RobotFeedback_getWheel_locked(rfp);
	rf->wheel_slipping      = RobotFeedback_getWheel_slipping(rfp);
	rf->rssi                = RobotFeedback_getRssi(rfp);
}



/** ================================ DECODE ================================ */
static inline void encodeRobotFeedback(RobotFeedbackPayload *rfp, RobotFeedback *rf){
	RobotFeedback_setHeader(rfp, rf->header);
	RobotFeedback_setId(rfp, rf->id);
	RobotFeedback_setBattery_level(rfp, rf->battery_level);
	RobotFeedback_setXsens_calibrated(rfp, rf->xsens_calibrated);
	RobotFeedback_setBallsensor_working(rfp, rf->ballsensor_working);
	RobotFeedback_setHas_ball(rfp, rf->has_ball);
	RobotFeedback_setCapacitor_charged(rfp, rf->capacitor_charged);
	RobotFeedback_setBall_position(rfp, rf->ball_position);
	RobotFeedback_setRho(rfp, rf->rho);
	RobotFeedback_setTheta(rfp, rf->theta);
	RobotFeedback_setAngle(rfp, rf->angle);
	RobotFeedback_setWheel_locked(rfp, rf->wheel_locked);
	RobotFeedback_setWheel_slipping(rfp, rf->wheel_slipping);
	RobotFeedback_setRssi(rfp, rf->rssi);
=======
                                //Description                           Values          Units     	Represented values      Bits
typedef struct _RobotFeedback {
    PACKET_TYPE header;         //Packet type (RobotFeedback)           -
    uint8_t id:8;               //Robot ID                              [0, 15]         -           [0, 15]                  8
    bool XsensCalibrated:1;     //XsensCalibrated                       [0, 1]          -           {false, true}            1
    bool battery:1;             //Battery empty                         [0, 1]          -           {false, true}            1
    bool ballSensorWorking:1;   //Ballsensor active and working         [0, 1]          -           {false, true}            1
    bool hasBall:1;             //Robot has ball                        [0, 1]          -           {false, true}            1
    uint8_t ballPos:4;          //Ballpos seen by sensor                [0, 15]                     {left, .., right}        4
    bool genevaWorking:1;       //Geneva active and working             [0, 1]          -           {false, true}            1
    uint8_t genevaState:7;      //Geneva oriëntation                    [0, 5]          -           {none, left, .. right}   7
    int16_t rho:11;             //Robot driving velocity                [0, 2047]       0.004m/s    [0, 8.188]              11
    int16_t angle:10;           //Robot driving direction               [-512, 511]     0.00614f    [-pi, pi>               10
    int16_t theta:11;           //Robot oriëntation                     [-1024, 1023]   0.00307rad  [-pi, pi]               11
    bool wheelBraking:1;        //Robot has a wheel braking             [0, 1]          -           {false, true}            1
    uint8_t rssi:7;             //Last received packet RSSI strength    [0, 127]        -1dBm       [0dBm, -127dBm]          7
} RobotFeedback;

// Packet structure
// [---0--] [---1--] [---2--] [---3--] [---4--] [---5--] [---6--] [---7--] [---8--]
// 11111111 -------- -------- -------- -------- -------- -------- -------- -------- // header
// -------- 11111111 -------- -------- -------- -------- -------- -------- -------- // id
// -------- -------- 1------- -------- -------- -------- -------- -------- -------- // XsensCalibrated
// -------- -------- -1------ -------- -------- -------- -------- -------- -------- // battery
// -------- -------- --1----- -------- -------- -------- -------- -------- -------- // ballSensorWorking
// -------- -------- ---1---- -------- -------- -------- -------- -------- -------- // hasBall
// -------- -------- ----1111 -------- -------- -------- -------- -------- -------- // ballPos
// -------- -------- -------- 1------- -------- -------- -------- -------- -------- // genevaWorking
// -------- -------- -------- -1111111 -------- -------- -------- -------- -------- // genevaState
// -------- -------- -------- -------- 11111111 111----- -------- -------- -------- // rho
// -------- -------- -------- -------- -------- ---11111 11111--- -------- -------- // angle
// -------- -------- -------- -------- -------- -------- -----111 11111111 -------- // theta
// -------- -------- -------- -------- -------- -------- -------- -------- 1------- // wheelBraking
// -------- -------- -------- -------- -------- -------- -------- -------- -1111111 // rssi

// -------------------------------------- GETTERS --------------------------------------
static inline PACKET_TYPE RobotFeedback_getHeader(RobotFeedbackPayload *rfp){
    return (PACKET_TYPE)rfp->payload[0];
}
static inline uint8_t RobotFeedback_getRobotId(RobotFeedbackPayload *rfp){
    return rfp->payload[1];
}
static inline bool RobotFeedback_getXsensCalibrated(RobotFeedbackPayload *rfp){
    return rfp->payload[2] & 0b10000000 ? true : false;
}
static inline bool RobotFeedback_getBattery(RobotFeedbackPayload *rfp){
    return rfp->payload[2] & 0b01000000 ? true : false;
}
static inline bool RobotFeedback_getBallSensorWorking(RobotFeedbackPayload *rfp){
    return rfp->payload[2] & 0b00100000 ? true : false;
}
static inline bool RobotFeedback_getHasBall(RobotFeedbackPayload *rfp){
    return rfp->payload[2] & 0b00010000 ? true : false;
}
static inline uint8_t RobotFeedback_getBallPos(RobotFeedbackPayload *rfp){
    return rfp->payload[2] & 0b00001111;
}
static inline bool RobotFeedback_getGenevaWorking(RobotFeedbackPayload *rfp){
    return rfp->payload[3] & 0b10000000 ? true : false;
}
static inline uint8_t RobotFeedback_getGenevaState(RobotFeedbackPayload *rfp){
    return rfp->payload[3] & 0b01111111;
}
static inline int16_t RobotFeedback_getRho(RobotFeedbackPayload *rfp){
    return ((rfp->payload[4] & 0b11111111) << 3)
         | ((rfp->payload[5] & 0b11100000) >> 5);
}
static inline int16_t RobotFeedback_getAngle(RobotFeedbackPayload *rfp){
    return ((rfp->payload[5] & 0b00011111) << 5)
         | ((rfp->payload[6] & 0b11111000) >> 3);
}
static inline int16_t RobotFeedback_getTheta(RobotFeedbackPayload *rfp){
    return ((rfp->payload[6] & 0b00000111) << 8)
         | ((rfp->payload[7] & 0b11111111) >> 0);
}
static inline bool RobotFeedback_getWheelBraking(RobotFeedbackPayload *rfp){
    return rfp->payload[8] & 0b10000000 ? true : false;
}
static inline uint8_t RobotFeedback_getRSSI(RobotFeedbackPayload *rfp){
    return rfp->payload[8] & 0b01111111;
}

// -------------------------------------- SETTERS --------------------------------------
// Note: functions assume an empty {0} packet to write to
static inline void RobotFeedback_setHeader(RobotFeedbackPayload *rcp, PACKET_TYPE header){
    rcp->payload[0] = header;
}
static inline void RobotFeedback_setId(RobotFeedbackPayload *rcp, uint8_t id){
    rcp->payload[1] = id;
}
static inline void RobotFeedback_setXsensCalibrated(RobotFeedbackPayload *rfp, bool XsensCalibrated){
    rfp->payload[2] |= (XsensCalibrated << 7) & 0b10000000;
}
static inline void RobotFeedback_setBattery(RobotFeedbackPayload *rfp, bool battery){
    rfp->payload[2] |= (battery << 6) & 0b01000000;
}
static inline void RobotFeedback_setBallSensorWorking(RobotFeedbackPayload *rfp, bool ballSensorWorking){
    rfp->payload[2] |= (ballSensorWorking << 6) & 0b00100000;
}
static inline void RobotFeedback_setHasBall(RobotFeedbackPayload *rfp, bool hasBall){
    rfp->payload[2] |= (hasBall << 5) & 0b00010000;
}
static inline void RobotFeedback_setBallPos(RobotFeedbackPayload *rfp, uint8_t ballPos){
    rfp->payload[2] |= (ballPos) & 0b00001111;
}
static inline void RobotFeedback_setGenevaWorking(RobotFeedbackPayload *rfp, bool genevaWorking){
    rfp->payload[3] |= (genevaWorking << 7) & 0b10000000;
}
static inline void RobotFeedback_setGenevaState(RobotFeedbackPayload *rfp, uint8_t genevaState){
    rfp->payload[3] |= (genevaState) & 0b01111111;
}
static inline void RobotFeedback_setRho(RobotFeedbackPayload *rfp, int16_t rho){
    rfp->payload[4]  = (rho >> 3) & 0b11111111;
    rfp->payload[5] |= (rho << 5) & 0b11100000;
}
static inline void RobotFeedback_setAngle(RobotFeedbackPayload *rfp, int16_t angle){
    rfp->payload[5] |= (angle >> 5) & 0b00011111;
    rfp->payload[6] |= (angle << 3) & 0b11111000;
}
static inline void RobotFeedback_setTheta(RobotFeedbackPayload *rfp, int16_t theta){
    rfp->payload[6] |= (theta >> 8) & 0b00000111;
    rfp->payload[7] |= (theta << 0) & 0b11111111;
}
static inline void RobotFeedback_setWheelBraking(RobotFeedbackPayload *rfp, bool wheelBraking){
    rfp->payload[8] |= (wheelBraking << 7) & 0b10000000;
}
static inline void RobotFeedback_setRSSI(RobotFeedbackPayload *rfp, uint8_t rssi){
    rfp->payload[8] |= (rssi) & 0b01111111;
}

// -------------------------------------- ENCODE/DECODE --------------------------------------
static inline void decodeRobotFeedback(RobotFeedback *feedback, RobotFeedbackPayload *rfp){
    feedback->header = RobotFeedback_getHeader(rfp);
    feedback->id = RobotFeedback_getRobotId(rfp);
    feedback->XsensCalibrated = RobotFeedback_getXsensCalibrated(rfp);
    feedback->battery = RobotFeedback_getBattery(rfp);
    feedback->ballSensorWorking = RobotFeedback_getBallSensorWorking(rfp);
    feedback->hasBall = RobotFeedback_getHasBall(rfp);
    feedback->ballPos = RobotFeedback_getBallPos(rfp);
    feedback->genevaWorking = RobotFeedback_getGenevaWorking(rfp);
    feedback->genevaState = RobotFeedback_getGenevaState(rfp);
    feedback->rho = RobotFeedback_getRho(rfp);
    feedback->angle = RobotFeedback_getAngle(rfp);
    feedback->theta = RobotFeedback_getTheta(rfp);
    feedback->wheelBraking = RobotFeedback_getWheelBraking(rfp);
    feedback->rssi = RobotFeedback_getRSSI(rfp);
>>>>>>> cacbe83ecd21cd0040f90252cc00a33dedd047f5
}

static inline void encodeRobotFeedback(RobotFeedbackPayload *rfp, RobotFeedback *feedback){
    RobotFeedback_setHeader(rfp, feedback->header);
    RobotFeedback_setId(rfp, feedback->id);
    RobotFeedback_setXsensCalibrated(rfp, feedback->XsensCalibrated);
    RobotFeedback_setBattery(rfp, feedback->battery);
    RobotFeedback_setBallSensorWorking(rfp, feedback->ballSensorWorking);
    RobotFeedback_setHasBall(rfp, feedback->hasBall);
    RobotFeedback_setBallPos(rfp, feedback->ballPos);
    RobotFeedback_setGenevaWorking(rfp, feedback->genevaWorking);
    RobotFeedback_setGenevaState(rfp, feedback->genevaState);
    RobotFeedback_setRho(rfp, feedback->rho);
    RobotFeedback_setAngle(rfp, feedback->angle);
    RobotFeedback_setTheta(rfp, feedback->theta);
    RobotFeedback_setWheelBraking(rfp, feedback->wheelBraking);
    RobotFeedback_setRSSI(rfp, feedback->rssi);
}
#endif /*__ROBOTFEEDBACK_H*/