#ifndef __ROBOTFEEDBACK_H
#define __ROBOTFEEDBACK_H

#include <stdbool.h>
#include <stdint.h>

#include "BaseTypes.h"

typedef struct _RobotFeedbackPayload {
    uint8_t payload[PACKET_SIZE_ROBOT_FEEDBACK];
} RobotFeedbackPayload;

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
inline PACKET_TYPE RobotFeedback_getHeader(RobotFeedbackPayload *rfp){
    return (PACKET_TYPE)rfp->payload[0];
}
inline uint8_t RobotFeedback_getRobotId(RobotFeedbackPayload *rfp){
    return rfp->payload[1];
}
inline bool RobotFeedback_getXsensCalibrated(RobotFeedbackPayload *rfp){
    return rfp->payload[2] & 0b10000000 ? true : false;
}
inline bool RobotFeedback_getBattery(RobotFeedbackPayload *rfp){
    return rfp->payload[2] & 0b01000000 ? true : false;
}
inline bool RobotFeedback_getBallSensorWorking(RobotFeedbackPayload *rfp){
    return rfp->payload[2] & 0b00100000 ? true : false;
}
inline bool RobotFeedback_getHasBall(RobotFeedbackPayload *rfp){
    return rfp->payload[2] & 0b00010000 ? true : false;
}
inline uint8_t RobotFeedback_getBallPos(RobotFeedbackPayload *rfp){
    return rfp->payload[2] & 0b00001111;
}
inline bool RobotFeedback_getGenevaWorking(RobotFeedbackPayload *rfp){
    return rfp->payload[3] & 0b10000000 ? true : false;
}
inline uint8_t RobotFeedback_getGenevaState(RobotFeedbackPayload *rfp){
    return rfp->payload[3] & 0b01111111;
}
inline int16_t RobotFeedback_getRho(RobotFeedbackPayload *rfp){
    return ((rfp->payload[4] & 0b11111111) << 3)
         | ((rfp->payload[5] & 0b11100000) >> 5);
}
inline int16_t RobotFeedback_getAngle(RobotFeedbackPayload *rfp){
    return ((rfp->payload[5] & 0b00011111) << 5)
         | ((rfp->payload[6] & 0b11111000) >> 3);
}
inline int16_t RobotFeedback_getTheta(RobotFeedbackPayload *rfp){
    return ((rfp->payload[6] & 0b00000111) << 8)
         | ((rfp->payload[7] & 0b11111111) >> 0);
}
inline bool RobotFeedback_getWheelBraking(RobotFeedbackPayload *rfp){
    return rfp->payload[8] & 0b10000000 ? true : false;
}
inline uint8_t RobotFeedback_getRSSI(RobotFeedbackPayload *rfp){
    return rfp->payload[8] & 0b01111111;
}

// -------------------------------------- SETTERS --------------------------------------
// Note: functions assume an empty {0} packet to write to
inline void RobotFeedback_setHeader(RobotFeedbackPayload *rcp, PACKET_TYPE header){
    rcp->payload[0] = header;
}
inline void RobotFeedback_setId(RobotFeedbackPayload *rcp, uint8_t id){
    rcp->payload[1] = id;
}
inline void RobotFeedback_setXsensCalibrated(RobotFeedbackPayload *rfp, bool XsensCalibrated){
    rfp->payload[2] |= (XsensCalibrated << 7) & 0b10000000;
}
inline void RobotFeedback_setBattery(RobotFeedbackPayload *rfp, bool battery){
    rfp->payload[2] |= (battery << 6) & 0b01000000;
}
inline void RobotFeedback_setBallSensorWorking(RobotFeedbackPayload *rfp, bool ballSensorWorking){
    rfp->payload[2] |= (ballSensorWorking << 6) & 0b00100000;
}
inline void RobotFeedback_setHasBall(RobotFeedbackPayload *rfp, bool hasBall){
    rfp->payload[2] |= (hasBall << 5) & 0b00010000;
}
inline void RobotFeedback_setBallPos(RobotFeedbackPayload *rfp, uint8_t ballPos){
    rfp->payload[2] |= (ballPos) & 0b00001111;
}
inline void RobotFeedback_setGenevaWorking(RobotFeedbackPayload *rfp, bool genevaWorking){
    rfp->payload[3] |= (genevaWorking << 7) & 0b10000000;
}
inline void RobotFeedback_setGenevaState(RobotFeedbackPayload *rfp, uint8_t genevaState){
    rfp->payload[3] |= (genevaState) & 0b01111111;
}
inline void RobotFeedback_setRho(RobotFeedbackPayload *rfp, int16_t rho){
    rfp->payload[4]  = (rho >> 3) & 0b11111111;
    rfp->payload[5] |= (rho << 5) & 0b11100000;
}
inline void RobotFeedback_setAngle(RobotFeedbackPayload *rfp, int16_t angle){
    rfp->payload[5] |= (angle >> 5) & 0b00011111;
    rfp->payload[6] |= (angle << 3) & 0b11111000;
}
inline void RobotFeedback_setTheta(RobotFeedbackPayload *rfp, int16_t theta){
    rfp->payload[6] |= (theta >> 8) & 0b00000111;
    rfp->payload[7] |= (theta << 0) & 0b11111111;
}
inline void RobotFeedback_setWheelBraking(RobotFeedbackPayload *rfp, bool wheelBraking){
    rfp->payload[8] |= (wheelBraking << 7) & 0b10000000;
}
inline void RobotFeedback_setRSSI(RobotFeedbackPayload *rfp, uint8_t rssi){
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