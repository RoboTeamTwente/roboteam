// AUTOGENERATED. Run generator/main.py to regenerate
/*
[  0   ] [  1   ]
11111111 -------- header
-------- 1111---- remVersion
*/

#ifndef __REM_BASESTATION_GET_CONFIGURATION_H
#define __REM_BASESTATION_GET_CONFIGURATION_H

#include <stdbool.h>
#include <stdint.h>
#include "REM_BaseTypes.h"

typedef struct _REM_BasestationGetConfigurationPayload {
    uint8_t payload[PACKET_SIZE_REM_BASESTATION_GET_CONFIGURATION];
} REM_BasestationGetConfigurationPayload;

typedef struct _REM_BasestationGetConfiguration {
    uint32_t   header              ; // integer [0, 255]             Header byte indicating the type of packet
    uint32_t   remVersion          ; // integer [0, 15]              Version of roboteam_embedded_messages
} REM_BasestationGetConfiguration;

// ================================ GETTERS ================================
static inline uint32_t REM_BasestationGetConfiguration_get_header(REM_BasestationGetConfigurationPayload *rembgcp){
    return ((rembgcp->payload[0]));
}

static inline uint32_t REM_BasestationGetConfiguration_get_remVersion(REM_BasestationGetConfigurationPayload *rembgcp){
    return ((rembgcp->payload[1] & 0b11110000) >> 4);
}

// ================================ SETTERS ================================
static inline void REM_BasestationGetConfiguration_set_header(REM_BasestationGetConfigurationPayload *rembgcp, uint32_t header){
    rembgcp->payload[0] = header;
}

static inline void REM_BasestationGetConfiguration_set_remVersion(REM_BasestationGetConfigurationPayload *rembgcp, uint32_t remVersion){
    rembgcp->payload[1] = ((remVersion << 4) & 0b11110000) | (rembgcp->payload[1] & 0b00001111);
}

// ================================ ENCODE ================================
static inline void encodeREM_BasestationGetConfiguration(REM_BasestationGetConfigurationPayload *rembgcp, REM_BasestationGetConfiguration *rembgc){
    REM_BasestationGetConfiguration_set_header              (rembgcp, rembgc->header);
    REM_BasestationGetConfiguration_set_remVersion          (rembgcp, rembgc->remVersion);
}

// ================================ DECODE ================================
static inline void decodeREM_BasestationGetConfiguration(REM_BasestationGetConfiguration *rembgc, REM_BasestationGetConfigurationPayload *rembgcp){
    rembgc->header       = REM_BasestationGetConfiguration_get_header(rembgcp);
    rembgc->remVersion   = REM_BasestationGetConfiguration_get_remVersion(rembgcp);
}

#endif /*__REM_BASESTATION_GET_CONFIGURATION_H*/
