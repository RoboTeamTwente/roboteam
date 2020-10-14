#ifndef __ROBOTFEEDBACK_H
#define __ROBOTFEEDBACK_H

#include <stdbool.h>
#include <stdint.h>

#include "BaseTypes.h"

typedef struct _RobotFeedbackPayload {
    uint8_t payload[PACKET_SIZE_ROBOT_FEEDBACK];
} RobotFeedbackPayload;

typedef struct _RobotFeedback {
    uint8_t header;
    uint8_t id;
    bool XsensCalibrated;
    bool battery;
    bool ballSensorWorking;
    bool hasBall;
    uint8_t ballPos;
    bool genevaWorking;
    uint8_t genevaState;
    int16_t rho;
    int16_t angle;
    int16_t theta;
    bool wheelBraking;
    uint8_t rssi;
} RobotFeedback;

inline uint8_t RobotFeedback_getHeader(RobotFeedbackPayload *rfp){
    return rfp->payload[0];
}
inline uint8_t RobotFeedback_getRobotId(RobotFeedbackPayload *rfp){
    return rfp->payload[1];
}
inline bool RobotFeedback_getXsensCalibrated(RobotFeedbackPayload *rfp){
    return rfp->payload[2] & 0b10000000;
}
inline bool RobotFeedback_getBattery(RobotFeedbackPayload *rfp){
    return rfp->payload[2] & 0b01000000;
}
inline bool RobotFeedback_getBallSensorWorking(RobotFeedbackPayload *rfp){
    return rfp->payload[2] & 0b00100000;
}
inline bool RobotFeedback_getHasBall(RobotFeedbackPayload *rfp){
    return rfp->payload[2] & 0b00010000;
}
inline uint8_t RobotFeedback_getBallPos(RobotFeedbackPayload *rfp){
    return rfp->payload[2] & 0b00001111;
}
inline bool RobotFeedback_getGenevaWorking(RobotFeedbackPayload *rfp){
    return rfp->payload[3] & 0b10000000;
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
    return rfp->payload[8] & 0b10000000;
}
inline uint8_t RobotFeedback_getRSSI(RobotFeedbackPayload *rfp){
    return rfp->payload[8] & 0b01111111;
}

static inline void fillRobotFeedback(RobotFeedback *feedback, RobotFeedbackPayload *rfp){
    feedback->header = RobotFeedback_getHeader(rfp);
    feedback->id = RobotFeedback_getRobotId(rfp);
    feedback->XsensCalibrated = RobotFeedback_getXsensCalibrated(rfp);
    feedback->battery = RobotFeedback_getBattery(rfp);
    feedback->ballSensorWorking = RobotFeedback_getBallSensorWorking(rfp);
    feedback->genevaWorking = RobotFeedback_getGenevaWorking(rfp);
    feedback->genevaState = RobotFeedback_getGenevaState(rfp);
    feedback->rho = RobotFeedback_getRho(rfp);
    feedback->angle = RobotFeedback_getAngle(rfp);
    feedback->theta = RobotFeedback_getTheta(rfp);
    feedback->wheelBraking = RobotFeedback_getWheelBraking(rfp);
    feedback->rssi = RobotFeedback_getRSSI(rfp);
}

#endif /*__ROBOTFEEDBACK_H*/