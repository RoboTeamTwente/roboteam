#ifndef __ROBOT_FEEDBACK_H
#define __ROBOT_FEEDBACK_H

#include <stdbool.h>
#include <stdint.h>

#include "BaseTypes.h"

typedef union _robotFeedback {
    uint8_t payload[PACKET_SIZE_ROBOT_FEEDBACK];
    struct __attribute__((__packed__)){
        PACKET_TYPE header:8;
        uint8_t roboID:8;
        bool	XsensCalibrated:1;
        bool	battery:1;
        bool	ballSensorWorking:1;
        bool	hasBall:1;
        uint8_t	ballPos:4;
        bool	genevaWorking:1;
        uint8_t	genevaState:7;
        int16_t	rho:11;
        int16_t	angle:10;
        int16_t	theta:11;
        bool	wheelBraking:1;
        uint8_t	signalStrength:7;
    };
} robotFeedback;

#endif /*__ROBOT_FEEDBACK_H*/