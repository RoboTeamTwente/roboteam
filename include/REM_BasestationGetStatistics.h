// AUTOGENERATED. Run generator/main.py to regenerate
/*
[  0   ]
11111111 header
*/

#ifndef __R_E_M__BASESTATION_GET_STATISTICS_H
#define __R_E_M__BASESTATION_GET_STATISTICS_H

#include <stdbool.h>
#include <stdint.h>
#include "BaseTypes.h"

typedef struct _REM_BasestationGetStatisticsPayload {
    uint8_t payload[PACKET_SIZE_R_E_M__BASESTATION_GET_STATISTICS];
} REM_BasestationGetStatisticsPayload;

typedef struct _REM_BasestationGetStatistics {
    uint32_t   header              ; // integer [0, 255]             Header byte indicating the type of packet
} REM_BasestationGetStatistics;

// ================================ GETTERS ================================
static inline uint32_t REM_BasestationGetStatistics_get_header(REM_BasestationGetStatisticsPayload *rembgsp){
    return ((rembgsp->payload[0]));
}

// ================================ SETTERS ================================
static inline void REM_BasestationGetStatistics_set_header(REM_BasestationGetStatisticsPayload *rembgsp, uint32_t header){
    rembgsp->payload[0] = header;
}

// ================================ ENCODE ================================
static inline void encodeREM_BasestationGetStatistics(REM_BasestationGetStatisticsPayload *rembgsp, REM_BasestationGetStatistics *rembgs){
    REM_BasestationGetStatistics_set_header              (rembgsp, rembgs->header);
}

// ================================ DECODE ================================
static inline void decodeREM_BasestationGetStatistics(REM_BasestationGetStatistics *rembgs, REM_BasestationGetStatisticsPayload *rembgsp){
    rembgs->header       = REM_BasestationGetStatistics_get_header(rembgsp);
}

#endif /*__R_E_M__BASESTATION_GET_STATISTICS_H*/
