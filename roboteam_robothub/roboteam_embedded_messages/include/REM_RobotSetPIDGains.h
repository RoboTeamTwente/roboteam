// AUTOGENERATED. Run generator/main.py to regenerate
/*
[  0   ] [  1   ] [  2   ] [  3   ] [  4   ] [  5   ] [  6   ] [  7   ] [  8   ] [  9   ] [  10  ] [  11  ] [  12  ] [  13  ] [  14  ] [  15  ] [  16  ] [  17  ] [  18  ] [  19  ] [  20  ] [  21  ] [  22  ] [  23  ] [  24  ] [  25  ] [  26  ] [  27  ] [  28  ] [  29  ] [  30  ] [  31  ] [  32  ] [  33  ] [  34  ] [  35  ] [  36  ] [  37  ]
11111111 -------- -------- -------- -------- -------- -------- -------- -------- -------- -------- -------- -------- -------- -------- -------- -------- -------- -------- -------- -------- -------- -------- -------- -------- -------- -------- -------- -------- -------- -------- -------- -------- -------- -------- -------- -------- -------- header
-------- 1111---- -------- -------- -------- -------- -------- -------- -------- -------- -------- -------- -------- -------- -------- -------- -------- -------- -------- -------- -------- -------- -------- -------- -------- -------- -------- -------- -------- -------- -------- -------- -------- -------- -------- -------- -------- -------- toRobotId
-------- ----1--- -------- -------- -------- -------- -------- -------- -------- -------- -------- -------- -------- -------- -------- -------- -------- -------- -------- -------- -------- -------- -------- -------- -------- -------- -------- -------- -------- -------- -------- -------- -------- -------- -------- -------- -------- -------- toColor
-------- -----1-- -------- -------- -------- -------- -------- -------- -------- -------- -------- -------- -------- -------- -------- -------- -------- -------- -------- -------- -------- -------- -------- -------- -------- -------- -------- -------- -------- -------- -------- -------- -------- -------- -------- -------- -------- -------- toBC
-------- ------1- -------- -------- -------- -------- -------- -------- -------- -------- -------- -------- -------- -------- -------- -------- -------- -------- -------- -------- -------- -------- -------- -------- -------- -------- -------- -------- -------- -------- -------- -------- -------- -------- -------- -------- -------- -------- toBS
-------- -------1 -------- -------- -------- -------- -------- -------- -------- -------- -------- -------- -------- -------- -------- -------- -------- -------- -------- -------- -------- -------- -------- -------- -------- -------- -------- -------- -------- -------- -------- -------- -------- -------- -------- -------- -------- -------- toPC
-------- -------- 1111---- -------- -------- -------- -------- -------- -------- -------- -------- -------- -------- -------- -------- -------- -------- -------- -------- -------- -------- -------- -------- -------- -------- -------- -------- -------- -------- -------- -------- -------- -------- -------- -------- -------- -------- -------- fromRobotId
-------- -------- ----1--- -------- -------- -------- -------- -------- -------- -------- -------- -------- -------- -------- -------- -------- -------- -------- -------- -------- -------- -------- -------- -------- -------- -------- -------- -------- -------- -------- -------- -------- -------- -------- -------- -------- -------- -------- fromColor
-------- -------- -----1-- -------- -------- -------- -------- -------- -------- -------- -------- -------- -------- -------- -------- -------- -------- -------- -------- -------- -------- -------- -------- -------- -------- -------- -------- -------- -------- -------- -------- -------- -------- -------- -------- -------- -------- -------- reserved
-------- -------- ------1- -------- -------- -------- -------- -------- -------- -------- -------- -------- -------- -------- -------- -------- -------- -------- -------- -------- -------- -------- -------- -------- -------- -------- -------- -------- -------- -------- -------- -------- -------- -------- -------- -------- -------- -------- fromBS
-------- -------- -------1 -------- -------- -------- -------- -------- -------- -------- -------- -------- -------- -------- -------- -------- -------- -------- -------- -------- -------- -------- -------- -------- -------- -------- -------- -------- -------- -------- -------- -------- -------- -------- -------- -------- -------- -------- fromPC
-------- -------- -------- 1111---- -------- -------- -------- -------- -------- -------- -------- -------- -------- -------- -------- -------- -------- -------- -------- -------- -------- -------- -------- -------- -------- -------- -------- -------- -------- -------- -------- -------- -------- -------- -------- -------- -------- -------- remVersion
-------- -------- -------- ----1111 -------- -------- -------- -------- -------- -------- -------- -------- -------- -------- -------- -------- -------- -------- -------- -------- -------- -------- -------- -------- -------- -------- -------- -------- -------- -------- -------- -------- -------- -------- -------- -------- -------- -------- messageId
-------- -------- -------- -------- 11111111 11111111 11111111 -------- -------- -------- -------- -------- -------- -------- -------- -------- -------- -------- -------- -------- -------- -------- -------- -------- -------- -------- -------- -------- -------- -------- -------- -------- -------- -------- -------- -------- -------- -------- timestamp
-------- -------- -------- -------- -------- -------- -------- 11111111 -------- -------- -------- -------- -------- -------- -------- -------- -------- -------- -------- -------- -------- -------- -------- -------- -------- -------- -------- -------- -------- -------- -------- -------- -------- -------- -------- -------- -------- -------- payloadSize
-------- -------- -------- -------- -------- -------- -------- -------- 11111111 11111111 -------- -------- -------- -------- -------- -------- -------- -------- -------- -------- -------- -------- -------- -------- -------- -------- -------- -------- -------- -------- -------- -------- -------- -------- -------- -------- -------- -------- PbodyX
-------- -------- -------- -------- -------- -------- -------- -------- -------- -------- 11111111 11111111 -------- -------- -------- -------- -------- -------- -------- -------- -------- -------- -------- -------- -------- -------- -------- -------- -------- -------- -------- -------- -------- -------- -------- -------- -------- -------- IbodyX
-------- -------- -------- -------- -------- -------- -------- -------- -------- -------- -------- -------- 11111111 11111111 -------- -------- -------- -------- -------- -------- -------- -------- -------- -------- -------- -------- -------- -------- -------- -------- -------- -------- -------- -------- -------- -------- -------- -------- DbodyX
-------- -------- -------- -------- -------- -------- -------- -------- -------- -------- -------- -------- -------- -------- 11111111 11111111 -------- -------- -------- -------- -------- -------- -------- -------- -------- -------- -------- -------- -------- -------- -------- -------- -------- -------- -------- -------- -------- -------- PbodyY
-------- -------- -------- -------- -------- -------- -------- -------- -------- -------- -------- -------- -------- -------- -------- -------- 11111111 11111111 -------- -------- -------- -------- -------- -------- -------- -------- -------- -------- -------- -------- -------- -------- -------- -------- -------- -------- -------- -------- IbodyY
-------- -------- -------- -------- -------- -------- -------- -------- -------- -------- -------- -------- -------- -------- -------- -------- -------- -------- 11111111 11111111 -------- -------- -------- -------- -------- -------- -------- -------- -------- -------- -------- -------- -------- -------- -------- -------- -------- -------- DbodyY
-------- -------- -------- -------- -------- -------- -------- -------- -------- -------- -------- -------- -------- -------- -------- -------- -------- -------- -------- -------- 11111111 11111111 -------- -------- -------- -------- -------- -------- -------- -------- -------- -------- -------- -------- -------- -------- -------- -------- PbodyW
-------- -------- -------- -------- -------- -------- -------- -------- -------- -------- -------- -------- -------- -------- -------- -------- -------- -------- -------- -------- -------- -------- 11111111 11111111 -------- -------- -------- -------- -------- -------- -------- -------- -------- -------- -------- -------- -------- -------- IbodyW
-------- -------- -------- -------- -------- -------- -------- -------- -------- -------- -------- -------- -------- -------- -------- -------- -------- -------- -------- -------- -------- -------- -------- -------- 11111111 11111111 -------- -------- -------- -------- -------- -------- -------- -------- -------- -------- -------- -------- DbodyW
-------- -------- -------- -------- -------- -------- -------- -------- -------- -------- -------- -------- -------- -------- -------- -------- -------- -------- -------- -------- -------- -------- -------- -------- -------- -------- 11111111 11111111 -------- -------- -------- -------- -------- -------- -------- -------- -------- -------- PbodyYaw
-------- -------- -------- -------- -------- -------- -------- -------- -------- -------- -------- -------- -------- -------- -------- -------- -------- -------- -------- -------- -------- -------- -------- -------- -------- -------- -------- -------- 11111111 11111111 -------- -------- -------- -------- -------- -------- -------- -------- IbodyYaw
-------- -------- -------- -------- -------- -------- -------- -------- -------- -------- -------- -------- -------- -------- -------- -------- -------- -------- -------- -------- -------- -------- -------- -------- -------- -------- -------- -------- -------- -------- 11111111 11111111 -------- -------- -------- -------- -------- -------- DbodyYaw
-------- -------- -------- -------- -------- -------- -------- -------- -------- -------- -------- -------- -------- -------- -------- -------- -------- -------- -------- -------- -------- -------- -------- -------- -------- -------- -------- -------- -------- -------- -------- -------- 11111111 11111111 -------- -------- -------- -------- Pwheels
-------- -------- -------- -------- -------- -------- -------- -------- -------- -------- -------- -------- -------- -------- -------- -------- -------- -------- -------- -------- -------- -------- -------- -------- -------- -------- -------- -------- -------- -------- -------- -------- -------- -------- 11111111 11111111 -------- -------- Iwheels
-------- -------- -------- -------- -------- -------- -------- -------- -------- -------- -------- -------- -------- -------- -------- -------- -------- -------- -------- -------- -------- -------- -------- -------- -------- -------- -------- -------- -------- -------- -------- -------- -------- -------- -------- -------- 11111111 11111111 Dwheels
*/

#ifndef __REM_ROBOT_SET_PIDGAINS_H
#define __REM_ROBOT_SET_PIDGAINS_H

#include <stdbool.h>
#include <stdint.h>
#include "REM_BaseTypes.h"

typedef struct _REM_RobotSetPIDGainsPayload {
    uint8_t payload[REM_PACKET_SIZE_REM_ROBOT_SET_PIDGAINS];
} REM_RobotSetPIDGainsPayload;

typedef struct _REM_RobotSetPIDGains {
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
    uint32_t   timestamp           ; // integer [0, 16777215]        Timestamp in milliseconds
    uint32_t   payloadSize         ; // integer [0, 255]             Size of the payload. At most 255 bytes including the generic_packet_header. Keep the 127 byte SX1280 limit in mind
    float      PbodyX              ; // float   [0.000, 40.000]      Commanded P gain of the PID for body_x (x-direction)
    float      IbodyX              ; // float   [0.000, 20.000]      Commanded I gain of the PID for body_x (x-direction)
    float      DbodyX              ; // float   [0.000, 10.000]      Commanded D gain of the PID for body_x (x-direction)
    float      PbodyY              ; // float   [0.000, 40.000]      Commanded P gain of the PID for body_y (y-direction)
    float      IbodyY              ; // float   [0.000, 20.000]      Commanded I gain of the PID for body_y (y-direction)
    float      DbodyY              ; // float   [0.000, 10.000]      Commanded D gain of the PID for body_y (y-direction)
    float      PbodyW              ; // float   [0.000, 40.000]      Commanded P gain of the PID for body_w (Angular velocity)
    float      IbodyW              ; // float   [0.000, 20.000]      Commanded I gain of the PID for body_w (Angular velocity)
    float      DbodyW              ; // float   [0.000, 10.000]      Commanded D gain of the PID for body_w (Angular velocity)
    float      PbodyYaw            ; // float   [0.000, 40.000]      Commanded P gain of the PID for body_yaw (Absolute angle)
    float      IbodyYaw            ; // float   [0.000, 20.000]      Commanded I gain of the PID for body_yaw (Absolute angle)
    float      DbodyYaw            ; // float   [0.000, 10.000]      Commanded D gain of the PID for body_yaw (Absolute angle)
    float      Pwheels             ; // float   [0.000, 40.000]      Commanded P gain of the PID for the wheels
    float      Iwheels             ; // float   [0.000, 20.000]      Commanded I gain of the PID for the wheels
    float      Dwheels             ; // float   [0.000, 10.000]      Commanded D gain of the PID for the wheels
} REM_RobotSetPIDGains;

// ================================ GETTERS ================================
static inline uint32_t REM_RobotSetPIDGains_get_header(REM_RobotSetPIDGainsPayload *remrspidgp){
    return ((remrspidgp->payload[0]));
}

static inline uint32_t REM_RobotSetPIDGains_get_toRobotId(REM_RobotSetPIDGainsPayload *remrspidgp){
    return ((remrspidgp->payload[1] & 0b11110000) >> 4);
}

static inline bool REM_RobotSetPIDGains_get_toColor(REM_RobotSetPIDGainsPayload *remrspidgp){
    return (remrspidgp->payload[1] & 0b00001000) > 0;
}

static inline bool REM_RobotSetPIDGains_get_toBC(REM_RobotSetPIDGainsPayload *remrspidgp){
    return (remrspidgp->payload[1] & 0b00000100) > 0;
}

static inline bool REM_RobotSetPIDGains_get_toBS(REM_RobotSetPIDGainsPayload *remrspidgp){
    return (remrspidgp->payload[1] & 0b00000010) > 0;
}

static inline bool REM_RobotSetPIDGains_get_toPC(REM_RobotSetPIDGainsPayload *remrspidgp){
    return (remrspidgp->payload[1] & 0b00000001) > 0;
}

static inline uint32_t REM_RobotSetPIDGains_get_fromRobotId(REM_RobotSetPIDGainsPayload *remrspidgp){
    return ((remrspidgp->payload[2] & 0b11110000) >> 4);
}

static inline bool REM_RobotSetPIDGains_get_fromColor(REM_RobotSetPIDGainsPayload *remrspidgp){
    return (remrspidgp->payload[2] & 0b00001000) > 0;
}

static inline bool REM_RobotSetPIDGains_get_reserved(REM_RobotSetPIDGainsPayload *remrspidgp){
    return (remrspidgp->payload[2] & 0b00000100) > 0;
}

static inline bool REM_RobotSetPIDGains_get_fromBS(REM_RobotSetPIDGainsPayload *remrspidgp){
    return (remrspidgp->payload[2] & 0b00000010) > 0;
}

static inline bool REM_RobotSetPIDGains_get_fromPC(REM_RobotSetPIDGainsPayload *remrspidgp){
    return (remrspidgp->payload[2] & 0b00000001) > 0;
}

static inline uint32_t REM_RobotSetPIDGains_get_remVersion(REM_RobotSetPIDGainsPayload *remrspidgp){
    return ((remrspidgp->payload[3] & 0b11110000) >> 4);
}

static inline uint32_t REM_RobotSetPIDGains_get_messageId(REM_RobotSetPIDGainsPayload *remrspidgp){
    return ((remrspidgp->payload[3] & 0b00001111));
}

static inline uint32_t REM_RobotSetPIDGains_get_timestamp(REM_RobotSetPIDGainsPayload *remrspidgp){
    return ((remrspidgp->payload[4]) << 16) | ((remrspidgp->payload[5]) << 8) | ((remrspidgp->payload[6]));
}

static inline uint32_t REM_RobotSetPIDGains_get_payloadSize(REM_RobotSetPIDGainsPayload *remrspidgp){
    return ((remrspidgp->payload[7]));
}

static inline float REM_RobotSetPIDGains_get_PbodyX(REM_RobotSetPIDGainsPayload *remrspidgp){
    uint32_t _PbodyX = ((remrspidgp->payload[8]) << 8) | ((remrspidgp->payload[9]));
    return (_PbodyX * 0.0006103608758679F);
}

static inline float REM_RobotSetPIDGains_get_IbodyX(REM_RobotSetPIDGainsPayload *remrspidgp){
    uint32_t _IbodyX = ((remrspidgp->payload[10]) << 8) | ((remrspidgp->payload[11]));
    return (_IbodyX * 0.0003051804379339F);
}

static inline float REM_RobotSetPIDGains_get_DbodyX(REM_RobotSetPIDGainsPayload *remrspidgp){
    uint32_t _DbodyX = ((remrspidgp->payload[12]) << 8) | ((remrspidgp->payload[13]));
    return (_DbodyX * 0.0001525902189670F);
}

static inline float REM_RobotSetPIDGains_get_PbodyY(REM_RobotSetPIDGainsPayload *remrspidgp){
    uint32_t _PbodyY = ((remrspidgp->payload[14]) << 8) | ((remrspidgp->payload[15]));
    return (_PbodyY * 0.0006103608758679F);
}

static inline float REM_RobotSetPIDGains_get_IbodyY(REM_RobotSetPIDGainsPayload *remrspidgp){
    uint32_t _IbodyY = ((remrspidgp->payload[16]) << 8) | ((remrspidgp->payload[17]));
    return (_IbodyY * 0.0003051804379339F);
}

static inline float REM_RobotSetPIDGains_get_DbodyY(REM_RobotSetPIDGainsPayload *remrspidgp){
    uint32_t _DbodyY = ((remrspidgp->payload[18]) << 8) | ((remrspidgp->payload[19]));
    return (_DbodyY * 0.0001525902189670F);
}

static inline float REM_RobotSetPIDGains_get_PbodyW(REM_RobotSetPIDGainsPayload *remrspidgp){
    uint32_t _PbodyW = ((remrspidgp->payload[20]) << 8) | ((remrspidgp->payload[21]));
    return (_PbodyW * 0.0006103608758679F);
}

static inline float REM_RobotSetPIDGains_get_IbodyW(REM_RobotSetPIDGainsPayload *remrspidgp){
    uint32_t _IbodyW = ((remrspidgp->payload[22]) << 8) | ((remrspidgp->payload[23]));
    return (_IbodyW * 0.0003051804379339F);
}

static inline float REM_RobotSetPIDGains_get_DbodyW(REM_RobotSetPIDGainsPayload *remrspidgp){
    uint32_t _DbodyW = ((remrspidgp->payload[24]) << 8) | ((remrspidgp->payload[25]));
    return (_DbodyW * 0.0001525902189670F);
}

static inline float REM_RobotSetPIDGains_get_PbodyYaw(REM_RobotSetPIDGainsPayload *remrspidgp){
    uint32_t _PbodyYaw = ((remrspidgp->payload[26]) << 8) | ((remrspidgp->payload[27]));
    return (_PbodyYaw * 0.0006103608758679F);
}

static inline float REM_RobotSetPIDGains_get_IbodyYaw(REM_RobotSetPIDGainsPayload *remrspidgp){
    uint32_t _IbodyYaw = ((remrspidgp->payload[28]) << 8) | ((remrspidgp->payload[29]));
    return (_IbodyYaw * 0.0003051804379339F);
}

static inline float REM_RobotSetPIDGains_get_DbodyYaw(REM_RobotSetPIDGainsPayload *remrspidgp){
    uint32_t _DbodyYaw = ((remrspidgp->payload[30]) << 8) | ((remrspidgp->payload[31]));
    return (_DbodyYaw * 0.0001525902189670F);
}

static inline float REM_RobotSetPIDGains_get_Pwheels(REM_RobotSetPIDGainsPayload *remrspidgp){
    uint32_t _Pwheels = ((remrspidgp->payload[32]) << 8) | ((remrspidgp->payload[33]));
    return (_Pwheels * 0.0006103608758679F);
}

static inline float REM_RobotSetPIDGains_get_Iwheels(REM_RobotSetPIDGainsPayload *remrspidgp){
    uint32_t _Iwheels = ((remrspidgp->payload[34]) << 8) | ((remrspidgp->payload[35]));
    return (_Iwheels * 0.0003051804379339F);
}

static inline float REM_RobotSetPIDGains_get_Dwheels(REM_RobotSetPIDGainsPayload *remrspidgp){
    uint32_t _Dwheels = ((remrspidgp->payload[36]) << 8) | ((remrspidgp->payload[37]));
    return (_Dwheels * 0.0001525902189670F);
}

// ================================ SETTERS ================================
static inline void REM_RobotSetPIDGains_set_header(REM_RobotSetPIDGainsPayload *remrspidgp, uint32_t header){
    remrspidgp->payload[0] = header;
}

static inline void REM_RobotSetPIDGains_set_toRobotId(REM_RobotSetPIDGainsPayload *remrspidgp, uint32_t toRobotId){
    remrspidgp->payload[1] = ((toRobotId << 4) & 0b11110000) | (remrspidgp->payload[1] & 0b00001111);
}

static inline void REM_RobotSetPIDGains_set_toColor(REM_RobotSetPIDGainsPayload *remrspidgp, bool toColor){
    remrspidgp->payload[1] = ((toColor << 3) & 0b00001000) | (remrspidgp->payload[1] & 0b11110111);
}

static inline void REM_RobotSetPIDGains_set_toBC(REM_RobotSetPIDGainsPayload *remrspidgp, bool toBC){
    remrspidgp->payload[1] = ((toBC << 2) & 0b00000100) | (remrspidgp->payload[1] & 0b11111011);
}

static inline void REM_RobotSetPIDGains_set_toBS(REM_RobotSetPIDGainsPayload *remrspidgp, bool toBS){
    remrspidgp->payload[1] = ((toBS << 1) & 0b00000010) | (remrspidgp->payload[1] & 0b11111101);
}

static inline void REM_RobotSetPIDGains_set_toPC(REM_RobotSetPIDGainsPayload *remrspidgp, bool toPC){
    remrspidgp->payload[1] = (toPC & 0b00000001) | (remrspidgp->payload[1] & 0b11111110);
}

static inline void REM_RobotSetPIDGains_set_fromRobotId(REM_RobotSetPIDGainsPayload *remrspidgp, uint32_t fromRobotId){
    remrspidgp->payload[2] = ((fromRobotId << 4) & 0b11110000) | (remrspidgp->payload[2] & 0b00001111);
}

static inline void REM_RobotSetPIDGains_set_fromColor(REM_RobotSetPIDGainsPayload *remrspidgp, bool fromColor){
    remrspidgp->payload[2] = ((fromColor << 3) & 0b00001000) | (remrspidgp->payload[2] & 0b11110111);
}

static inline void REM_RobotSetPIDGains_set_reserved(REM_RobotSetPIDGainsPayload *remrspidgp, bool reserved){
    remrspidgp->payload[2] = ((reserved << 2) & 0b00000100) | (remrspidgp->payload[2] & 0b11111011);
}

static inline void REM_RobotSetPIDGains_set_fromBS(REM_RobotSetPIDGainsPayload *remrspidgp, bool fromBS){
    remrspidgp->payload[2] = ((fromBS << 1) & 0b00000010) | (remrspidgp->payload[2] & 0b11111101);
}

static inline void REM_RobotSetPIDGains_set_fromPC(REM_RobotSetPIDGainsPayload *remrspidgp, bool fromPC){
    remrspidgp->payload[2] = (fromPC & 0b00000001) | (remrspidgp->payload[2] & 0b11111110);
}

static inline void REM_RobotSetPIDGains_set_remVersion(REM_RobotSetPIDGainsPayload *remrspidgp, uint32_t remVersion){
    remrspidgp->payload[3] = ((remVersion << 4) & 0b11110000) | (remrspidgp->payload[3] & 0b00001111);
}

static inline void REM_RobotSetPIDGains_set_messageId(REM_RobotSetPIDGainsPayload *remrspidgp, uint32_t messageId){
    remrspidgp->payload[3] = (messageId & 0b00001111) | (remrspidgp->payload[3] & 0b11110000);
}

static inline void REM_RobotSetPIDGains_set_timestamp(REM_RobotSetPIDGainsPayload *remrspidgp, uint32_t timestamp){
    remrspidgp->payload[4] = (timestamp >> 16);
    remrspidgp->payload[5] = (timestamp >> 8);
    remrspidgp->payload[6] = timestamp;
}

static inline void REM_RobotSetPIDGains_set_payloadSize(REM_RobotSetPIDGainsPayload *remrspidgp, uint32_t payloadSize){
    remrspidgp->payload[7] = payloadSize;
}

static inline void REM_RobotSetPIDGains_set_PbodyX(REM_RobotSetPIDGainsPayload *remrspidgp, float PbodyX){
    uint32_t _PbodyX = (uint32_t)(PbodyX / 0.0006103608758679F);
    remrspidgp->payload[8] = (_PbodyX >> 8);
    remrspidgp->payload[9] = _PbodyX;
}

static inline void REM_RobotSetPIDGains_set_IbodyX(REM_RobotSetPIDGainsPayload *remrspidgp, float IbodyX){
    uint32_t _IbodyX = (uint32_t)(IbodyX / 0.0003051804379339F);
    remrspidgp->payload[10] = (_IbodyX >> 8);
    remrspidgp->payload[11] = _IbodyX;
}

static inline void REM_RobotSetPIDGains_set_DbodyX(REM_RobotSetPIDGainsPayload *remrspidgp, float DbodyX){
    uint32_t _DbodyX = (uint32_t)(DbodyX / 0.0001525902189670F);
    remrspidgp->payload[12] = (_DbodyX >> 8);
    remrspidgp->payload[13] = _DbodyX;
}

static inline void REM_RobotSetPIDGains_set_PbodyY(REM_RobotSetPIDGainsPayload *remrspidgp, float PbodyY){
    uint32_t _PbodyY = (uint32_t)(PbodyY / 0.0006103608758679F);
    remrspidgp->payload[14] = (_PbodyY >> 8);
    remrspidgp->payload[15] = _PbodyY;
}

static inline void REM_RobotSetPIDGains_set_IbodyY(REM_RobotSetPIDGainsPayload *remrspidgp, float IbodyY){
    uint32_t _IbodyY = (uint32_t)(IbodyY / 0.0003051804379339F);
    remrspidgp->payload[16] = (_IbodyY >> 8);
    remrspidgp->payload[17] = _IbodyY;
}

static inline void REM_RobotSetPIDGains_set_DbodyY(REM_RobotSetPIDGainsPayload *remrspidgp, float DbodyY){
    uint32_t _DbodyY = (uint32_t)(DbodyY / 0.0001525902189670F);
    remrspidgp->payload[18] = (_DbodyY >> 8);
    remrspidgp->payload[19] = _DbodyY;
}

static inline void REM_RobotSetPIDGains_set_PbodyW(REM_RobotSetPIDGainsPayload *remrspidgp, float PbodyW){
    uint32_t _PbodyW = (uint32_t)(PbodyW / 0.0006103608758679F);
    remrspidgp->payload[20] = (_PbodyW >> 8);
    remrspidgp->payload[21] = _PbodyW;
}

static inline void REM_RobotSetPIDGains_set_IbodyW(REM_RobotSetPIDGainsPayload *remrspidgp, float IbodyW){
    uint32_t _IbodyW = (uint32_t)(IbodyW / 0.0003051804379339F);
    remrspidgp->payload[22] = (_IbodyW >> 8);
    remrspidgp->payload[23] = _IbodyW;
}

static inline void REM_RobotSetPIDGains_set_DbodyW(REM_RobotSetPIDGainsPayload *remrspidgp, float DbodyW){
    uint32_t _DbodyW = (uint32_t)(DbodyW / 0.0001525902189670F);
    remrspidgp->payload[24] = (_DbodyW >> 8);
    remrspidgp->payload[25] = _DbodyW;
}

static inline void REM_RobotSetPIDGains_set_PbodyYaw(REM_RobotSetPIDGainsPayload *remrspidgp, float PbodyYaw){
    uint32_t _PbodyYaw = (uint32_t)(PbodyYaw / 0.0006103608758679F);
    remrspidgp->payload[26] = (_PbodyYaw >> 8);
    remrspidgp->payload[27] = _PbodyYaw;
}

static inline void REM_RobotSetPIDGains_set_IbodyYaw(REM_RobotSetPIDGainsPayload *remrspidgp, float IbodyYaw){
    uint32_t _IbodyYaw = (uint32_t)(IbodyYaw / 0.0003051804379339F);
    remrspidgp->payload[28] = (_IbodyYaw >> 8);
    remrspidgp->payload[29] = _IbodyYaw;
}

static inline void REM_RobotSetPIDGains_set_DbodyYaw(REM_RobotSetPIDGainsPayload *remrspidgp, float DbodyYaw){
    uint32_t _DbodyYaw = (uint32_t)(DbodyYaw / 0.0001525902189670F);
    remrspidgp->payload[30] = (_DbodyYaw >> 8);
    remrspidgp->payload[31] = _DbodyYaw;
}

static inline void REM_RobotSetPIDGains_set_Pwheels(REM_RobotSetPIDGainsPayload *remrspidgp, float Pwheels){
    uint32_t _Pwheels = (uint32_t)(Pwheels / 0.0006103608758679F);
    remrspidgp->payload[32] = (_Pwheels >> 8);
    remrspidgp->payload[33] = _Pwheels;
}

static inline void REM_RobotSetPIDGains_set_Iwheels(REM_RobotSetPIDGainsPayload *remrspidgp, float Iwheels){
    uint32_t _Iwheels = (uint32_t)(Iwheels / 0.0003051804379339F);
    remrspidgp->payload[34] = (_Iwheels >> 8);
    remrspidgp->payload[35] = _Iwheels;
}

static inline void REM_RobotSetPIDGains_set_Dwheels(REM_RobotSetPIDGainsPayload *remrspidgp, float Dwheels){
    uint32_t _Dwheels = (uint32_t)(Dwheels / 0.0001525902189670F);
    remrspidgp->payload[36] = (_Dwheels >> 8);
    remrspidgp->payload[37] = _Dwheels;
}

// ================================ ENCODE ================================
static inline void encodeREM_RobotSetPIDGains(REM_RobotSetPIDGainsPayload *remrspidgp, REM_RobotSetPIDGains *remrspidg){
    REM_RobotSetPIDGains_set_header              (remrspidgp, remrspidg->header);
    REM_RobotSetPIDGains_set_toRobotId           (remrspidgp, remrspidg->toRobotId);
    REM_RobotSetPIDGains_set_toColor             (remrspidgp, remrspidg->toColor);
    REM_RobotSetPIDGains_set_toBC                (remrspidgp, remrspidg->toBC);
    REM_RobotSetPIDGains_set_toBS                (remrspidgp, remrspidg->toBS);
    REM_RobotSetPIDGains_set_toPC                (remrspidgp, remrspidg->toPC);
    REM_RobotSetPIDGains_set_fromRobotId         (remrspidgp, remrspidg->fromRobotId);
    REM_RobotSetPIDGains_set_fromColor           (remrspidgp, remrspidg->fromColor);
    REM_RobotSetPIDGains_set_reserved            (remrspidgp, remrspidg->reserved);
    REM_RobotSetPIDGains_set_fromBS              (remrspidgp, remrspidg->fromBS);
    REM_RobotSetPIDGains_set_fromPC              (remrspidgp, remrspidg->fromPC);
    REM_RobotSetPIDGains_set_remVersion          (remrspidgp, remrspidg->remVersion);
    REM_RobotSetPIDGains_set_messageId           (remrspidgp, remrspidg->messageId);
    REM_RobotSetPIDGains_set_timestamp           (remrspidgp, remrspidg->timestamp);
    REM_RobotSetPIDGains_set_payloadSize         (remrspidgp, remrspidg->payloadSize);
    REM_RobotSetPIDGains_set_PbodyX              (remrspidgp, remrspidg->PbodyX);
    REM_RobotSetPIDGains_set_IbodyX              (remrspidgp, remrspidg->IbodyX);
    REM_RobotSetPIDGains_set_DbodyX              (remrspidgp, remrspidg->DbodyX);
    REM_RobotSetPIDGains_set_PbodyY              (remrspidgp, remrspidg->PbodyY);
    REM_RobotSetPIDGains_set_IbodyY              (remrspidgp, remrspidg->IbodyY);
    REM_RobotSetPIDGains_set_DbodyY              (remrspidgp, remrspidg->DbodyY);
    REM_RobotSetPIDGains_set_PbodyW              (remrspidgp, remrspidg->PbodyW);
    REM_RobotSetPIDGains_set_IbodyW              (remrspidgp, remrspidg->IbodyW);
    REM_RobotSetPIDGains_set_DbodyW              (remrspidgp, remrspidg->DbodyW);
    REM_RobotSetPIDGains_set_PbodyYaw            (remrspidgp, remrspidg->PbodyYaw);
    REM_RobotSetPIDGains_set_IbodyYaw            (remrspidgp, remrspidg->IbodyYaw);
    REM_RobotSetPIDGains_set_DbodyYaw            (remrspidgp, remrspidg->DbodyYaw);
    REM_RobotSetPIDGains_set_Pwheels             (remrspidgp, remrspidg->Pwheels);
    REM_RobotSetPIDGains_set_Iwheels             (remrspidgp, remrspidg->Iwheels);
    REM_RobotSetPIDGains_set_Dwheels             (remrspidgp, remrspidg->Dwheels);
}

// ================================ DECODE ================================
static inline void decodeREM_RobotSetPIDGains(REM_RobotSetPIDGains *remrspidg, REM_RobotSetPIDGainsPayload *remrspidgp){
    remrspidg->header    = REM_RobotSetPIDGains_get_header(remrspidgp);
    remrspidg->toRobotId = REM_RobotSetPIDGains_get_toRobotId(remrspidgp);
    remrspidg->toColor   = REM_RobotSetPIDGains_get_toColor(remrspidgp);
    remrspidg->toBC      = REM_RobotSetPIDGains_get_toBC(remrspidgp);
    remrspidg->toBS      = REM_RobotSetPIDGains_get_toBS(remrspidgp);
    remrspidg->toPC      = REM_RobotSetPIDGains_get_toPC(remrspidgp);
    remrspidg->fromRobotId= REM_RobotSetPIDGains_get_fromRobotId(remrspidgp);
    remrspidg->fromColor = REM_RobotSetPIDGains_get_fromColor(remrspidgp);
    remrspidg->reserved  = REM_RobotSetPIDGains_get_reserved(remrspidgp);
    remrspidg->fromBS    = REM_RobotSetPIDGains_get_fromBS(remrspidgp);
    remrspidg->fromPC    = REM_RobotSetPIDGains_get_fromPC(remrspidgp);
    remrspidg->remVersion= REM_RobotSetPIDGains_get_remVersion(remrspidgp);
    remrspidg->messageId = REM_RobotSetPIDGains_get_messageId(remrspidgp);
    remrspidg->timestamp = REM_RobotSetPIDGains_get_timestamp(remrspidgp);
    remrspidg->payloadSize= REM_RobotSetPIDGains_get_payloadSize(remrspidgp);
    remrspidg->PbodyX    = REM_RobotSetPIDGains_get_PbodyX(remrspidgp);
    remrspidg->IbodyX    = REM_RobotSetPIDGains_get_IbodyX(remrspidgp);
    remrspidg->DbodyX    = REM_RobotSetPIDGains_get_DbodyX(remrspidgp);
    remrspidg->PbodyY    = REM_RobotSetPIDGains_get_PbodyY(remrspidgp);
    remrspidg->IbodyY    = REM_RobotSetPIDGains_get_IbodyY(remrspidgp);
    remrspidg->DbodyY    = REM_RobotSetPIDGains_get_DbodyY(remrspidgp);
    remrspidg->PbodyW    = REM_RobotSetPIDGains_get_PbodyW(remrspidgp);
    remrspidg->IbodyW    = REM_RobotSetPIDGains_get_IbodyW(remrspidgp);
    remrspidg->DbodyW    = REM_RobotSetPIDGains_get_DbodyW(remrspidgp);
    remrspidg->PbodyYaw  = REM_RobotSetPIDGains_get_PbodyYaw(remrspidgp);
    remrspidg->IbodyYaw  = REM_RobotSetPIDGains_get_IbodyYaw(remrspidgp);
    remrspidg->DbodyYaw  = REM_RobotSetPIDGains_get_DbodyYaw(remrspidgp);
    remrspidg->Pwheels   = REM_RobotSetPIDGains_get_Pwheels(remrspidgp);
    remrspidg->Iwheels   = REM_RobotSetPIDGains_get_Iwheels(remrspidgp);
    remrspidg->Dwheels   = REM_RobotSetPIDGains_get_Dwheels(remrspidgp);
}

#endif /*__REM_ROBOT_SET_PIDGAINS_H*/