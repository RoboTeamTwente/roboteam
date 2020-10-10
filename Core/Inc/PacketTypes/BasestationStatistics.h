#ifndef __BASESTATION_STATISTICS_H
#define __BASESTATION_STATISTICS_H

#include <stdbool.h>
#include <stdint.h>

#include "BaseTypes.h"

typedef union _basestationStatistics{
    uint8_t payload[PACKET_SIZE_BASESTATION_STATISTICS];
    struct{
        uint8_t header:8;
        struct{
            uint8_t packetsReceived:8;
            uint8_t packetsSent:8;
        } robot[16];
    };
} basestationStatistics;

#endif /*__BASESTATION_STATISTICS_H*/