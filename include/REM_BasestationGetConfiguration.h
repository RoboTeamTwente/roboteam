// AUTOGENERATED. Run generator/main.py to regenerate
/*
[  0   ] [  1   ] [  2   ] [  3   ] [  4   ] [  5   ]
11111111 -------- -------- -------- -------- -------- header
-------- 1111---- -------- -------- -------- -------- toRobotId
-------- ----1--- -------- -------- -------- -------- toColor
-------- -----1-- -------- -------- -------- -------- toBC
-------- ------1- -------- -------- -------- -------- toBS
-------- -------1 -------- -------- -------- -------- toPC
-------- -------- 1111---- -------- -------- -------- fromRobotId
-------- -------- ----1--- -------- -------- -------- fromColor
-------- -------- -----1-- -------- -------- -------- reserved
-------- -------- ------1- -------- -------- -------- fromBS
-------- -------- -------1 -------- -------- -------- fromPC
-------- -------- -------- 1111---- -------- -------- remVersion
-------- -------- -------- ----1111 -------- -------- messageId
-------- -------- -------- -------- 11111111 -------- payloadSize
-------- -------- -------- -------- -------- 11111111 header
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
    uint32_t   toRobotId           ; // integer [0, 15]              Id of the receiving robot
    bool       toColor             ; // integer [0, 1]               Color of the receiving robot / basestation. Yellow = 0, Blue = 1
    bool       toBC                ; // integer [0, 1]               Bit indicating this packet has to be broadcasted to all robots
    bool       toBS                ; // integer [0, 1]               Bit indicating this packet is meant for the basestation
    bool       toPC                ; // integer [0, 1]               Bit indicating this packet is meant for the PC
    uint32_t   fromRobotId         ; // integer [0, 15]              Id of the transmitting robot
    bool       fromColor           ; // integer [0, 1]               Color of the transmitting robot / basestation. Yellow = 0, Blue = 1
    bool       reserved            ; // integer [0, 1]               reserved
    bool       fromBS              ; // integer [0, 1]               Bit indicating this packet is coming from the basestation
    bool       fromPC              ; // integer [0, 1]               Bit indicating this packet is coming from the PC
    uint32_t   remVersion          ; // integer [0, 15]              Version of roboteam_embedded_messages
    uint32_t   messageId           ; // integer [0, 15]              messageId. Can be used for aligning packets
    uint32_t   payloadSize         ; // integer [0, 255]             Size of the payload. At most 255 bytes including the generic_packet_header. Keep the 127 byte SX1280 limit in mind
    uint32_t   header              ; // integer [0, 255]             Header byte indicating the type of packet
} REM_BasestationGetConfiguration;

// ================================ GETTERS ================================
static inline uint32_t REM_BasestationGetConfiguration_get_header(REM_BasestationGetConfigurationPayload *rembgcp){
    return ((rembgcp->payload[0]));
}

static inline uint32_t REM_BasestationGetConfiguration_get_toRobotId(REM_BasestationGetConfigurationPayload *rembgcp){
    return ((rembgcp->payload[1] & 0b11110000) >> 4);
}

static inline bool REM_BasestationGetConfiguration_get_toColor(REM_BasestationGetConfigurationPayload *rembgcp){
    return (rembgcp->payload[1] & 0b00001000) > 0;
}

static inline bool REM_BasestationGetConfiguration_get_toBC(REM_BasestationGetConfigurationPayload *rembgcp){
    return (rembgcp->payload[1] & 0b00000100) > 0;
}

static inline bool REM_BasestationGetConfiguration_get_toBS(REM_BasestationGetConfigurationPayload *rembgcp){
    return (rembgcp->payload[1] & 0b00000010) > 0;
}

static inline bool REM_BasestationGetConfiguration_get_toPC(REM_BasestationGetConfigurationPayload *rembgcp){
    return (rembgcp->payload[1] & 0b00000001) > 0;
}

static inline uint32_t REM_BasestationGetConfiguration_get_fromRobotId(REM_BasestationGetConfigurationPayload *rembgcp){
    return ((rembgcp->payload[2] & 0b11110000) >> 4);
}

static inline bool REM_BasestationGetConfiguration_get_fromColor(REM_BasestationGetConfigurationPayload *rembgcp){
    return (rembgcp->payload[2] & 0b00001000) > 0;
}

static inline bool REM_BasestationGetConfiguration_get_reserved(REM_BasestationGetConfigurationPayload *rembgcp){
    return (rembgcp->payload[2] & 0b00000100) > 0;
}

static inline bool REM_BasestationGetConfiguration_get_fromBS(REM_BasestationGetConfigurationPayload *rembgcp){
    return (rembgcp->payload[2] & 0b00000010) > 0;
}

static inline bool REM_BasestationGetConfiguration_get_fromPC(REM_BasestationGetConfigurationPayload *rembgcp){
    return (rembgcp->payload[2] & 0b00000001) > 0;
}

static inline uint32_t REM_BasestationGetConfiguration_get_remVersion(REM_BasestationGetConfigurationPayload *rembgcp){
    return ((rembgcp->payload[3] & 0b11110000) >> 4);
}

static inline uint32_t REM_BasestationGetConfiguration_get_messageId(REM_BasestationGetConfigurationPayload *rembgcp){
    return ((rembgcp->payload[3] & 0b00001111));
}

static inline uint32_t REM_BasestationGetConfiguration_get_payloadSize(REM_BasestationGetConfigurationPayload *rembgcp){
    return ((rembgcp->payload[4]));
}

static inline uint32_t REM_BasestationGetConfiguration_get_header(REM_BasestationGetConfigurationPayload *rembgcp){
    return ((rembgcp->payload[5]));
}

// ================================ SETTERS ================================
static inline void REM_BasestationGetConfiguration_set_header(REM_BasestationGetConfigurationPayload *rembgcp, uint32_t header){
    rembgcp->payload[0] = header;
}

static inline void REM_BasestationGetConfiguration_set_toRobotId(REM_BasestationGetConfigurationPayload *rembgcp, uint32_t toRobotId){
    rembgcp->payload[1] = ((toRobotId << 4) & 0b11110000) | (rembgcp->payload[1] & 0b00001111);
}

static inline void REM_BasestationGetConfiguration_set_toColor(REM_BasestationGetConfigurationPayload *rembgcp, bool toColor){
    rembgcp->payload[1] = ((toColor << 3) & 0b00001000) | (rembgcp->payload[1] & 0b11110111);
}

static inline void REM_BasestationGetConfiguration_set_toBC(REM_BasestationGetConfigurationPayload *rembgcp, bool toBC){
    rembgcp->payload[1] = ((toBC << 2) & 0b00000100) | (rembgcp->payload[1] & 0b11111011);
}

static inline void REM_BasestationGetConfiguration_set_toBS(REM_BasestationGetConfigurationPayload *rembgcp, bool toBS){
    rembgcp->payload[1] = ((toBS << 1) & 0b00000010) | (rembgcp->payload[1] & 0b11111101);
}

static inline void REM_BasestationGetConfiguration_set_toPC(REM_BasestationGetConfigurationPayload *rembgcp, bool toPC){
    rembgcp->payload[1] = (toPC & 0b00000001) | (rembgcp->payload[1] & 0b11111110);
}

static inline void REM_BasestationGetConfiguration_set_fromRobotId(REM_BasestationGetConfigurationPayload *rembgcp, uint32_t fromRobotId){
    rembgcp->payload[2] = ((fromRobotId << 4) & 0b11110000) | (rembgcp->payload[2] & 0b00001111);
}

static inline void REM_BasestationGetConfiguration_set_fromColor(REM_BasestationGetConfigurationPayload *rembgcp, bool fromColor){
    rembgcp->payload[2] = ((fromColor << 3) & 0b00001000) | (rembgcp->payload[2] & 0b11110111);
}

static inline void REM_BasestationGetConfiguration_set_reserved(REM_BasestationGetConfigurationPayload *rembgcp, bool reserved){
    rembgcp->payload[2] = ((reserved << 2) & 0b00000100) | (rembgcp->payload[2] & 0b11111011);
}

static inline void REM_BasestationGetConfiguration_set_fromBS(REM_BasestationGetConfigurationPayload *rembgcp, bool fromBS){
    rembgcp->payload[2] = ((fromBS << 1) & 0b00000010) | (rembgcp->payload[2] & 0b11111101);
}

static inline void REM_BasestationGetConfiguration_set_fromPC(REM_BasestationGetConfigurationPayload *rembgcp, bool fromPC){
    rembgcp->payload[2] = (fromPC & 0b00000001) | (rembgcp->payload[2] & 0b11111110);
}

static inline void REM_BasestationGetConfiguration_set_remVersion(REM_BasestationGetConfigurationPayload *rembgcp, uint32_t remVersion){
    rembgcp->payload[3] = ((remVersion << 4) & 0b11110000) | (rembgcp->payload[3] & 0b00001111);
}

static inline void REM_BasestationGetConfiguration_set_messageId(REM_BasestationGetConfigurationPayload *rembgcp, uint32_t messageId){
    rembgcp->payload[3] = (messageId & 0b00001111) | (rembgcp->payload[3] & 0b11110000);
}

static inline void REM_BasestationGetConfiguration_set_payloadSize(REM_BasestationGetConfigurationPayload *rembgcp, uint32_t payloadSize){
    rembgcp->payload[4] = payloadSize;
}

static inline void REM_BasestationGetConfiguration_set_header(REM_BasestationGetConfigurationPayload *rembgcp, uint32_t header){
    rembgcp->payload[5] = header;
}

// ================================ ENCODE ================================
static inline void encodeREM_BasestationGetConfiguration(REM_BasestationGetConfigurationPayload *rembgcp, REM_BasestationGetConfiguration *rembgc){
    REM_BasestationGetConfiguration_set_header              (rembgcp, rembgc->header);
    REM_BasestationGetConfiguration_set_toRobotId           (rembgcp, rembgc->toRobotId);
    REM_BasestationGetConfiguration_set_toColor             (rembgcp, rembgc->toColor);
    REM_BasestationGetConfiguration_set_toBC                (rembgcp, rembgc->toBC);
    REM_BasestationGetConfiguration_set_toBS                (rembgcp, rembgc->toBS);
    REM_BasestationGetConfiguration_set_toPC                (rembgcp, rembgc->toPC);
    REM_BasestationGetConfiguration_set_fromRobotId         (rembgcp, rembgc->fromRobotId);
    REM_BasestationGetConfiguration_set_fromColor           (rembgcp, rembgc->fromColor);
    REM_BasestationGetConfiguration_set_reserved            (rembgcp, rembgc->reserved);
    REM_BasestationGetConfiguration_set_fromBS              (rembgcp, rembgc->fromBS);
    REM_BasestationGetConfiguration_set_fromPC              (rembgcp, rembgc->fromPC);
    REM_BasestationGetConfiguration_set_remVersion          (rembgcp, rembgc->remVersion);
    REM_BasestationGetConfiguration_set_messageId           (rembgcp, rembgc->messageId);
    REM_BasestationGetConfiguration_set_payloadSize         (rembgcp, rembgc->payloadSize);
    REM_BasestationGetConfiguration_set_header              (rembgcp, rembgc->header);
}

// ================================ DECODE ================================
static inline void decodeREM_BasestationGetConfiguration(REM_BasestationGetConfiguration *rembgc, REM_BasestationGetConfigurationPayload *rembgcp){
    rembgc->header       = REM_BasestationGetConfiguration_get_header(rembgcp);
    rembgc->toRobotId    = REM_BasestationGetConfiguration_get_toRobotId(rembgcp);
    rembgc->toColor      = REM_BasestationGetConfiguration_get_toColor(rembgcp);
    rembgc->toBC         = REM_BasestationGetConfiguration_get_toBC(rembgcp);
    rembgc->toBS         = REM_BasestationGetConfiguration_get_toBS(rembgcp);
    rembgc->toPC         = REM_BasestationGetConfiguration_get_toPC(rembgcp);
    rembgc->fromRobotId  = REM_BasestationGetConfiguration_get_fromRobotId(rembgcp);
    rembgc->fromColor    = REM_BasestationGetConfiguration_get_fromColor(rembgcp);
    rembgc->reserved     = REM_BasestationGetConfiguration_get_reserved(rembgcp);
    rembgc->fromBS       = REM_BasestationGetConfiguration_get_fromBS(rembgcp);
    rembgc->fromPC       = REM_BasestationGetConfiguration_get_fromPC(rembgcp);
    rembgc->remVersion   = REM_BasestationGetConfiguration_get_remVersion(rembgcp);
    rembgc->messageId    = REM_BasestationGetConfiguration_get_messageId(rembgcp);
    rembgc->payloadSize  = REM_BasestationGetConfiguration_get_payloadSize(rembgcp);
    rembgc->header       = REM_BasestationGetConfiguration_get_header(rembgcp);
}

#endif /*__REM_BASESTATION_GET_CONFIGURATION_H*/
