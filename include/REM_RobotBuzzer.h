// AUTOGENERATED. Run generator/main.py to regenerate
/*
[  0   ] [  1   ] [  2   ] [  3   ] [  4   ] [  5   ]
11111111 -------- -------- -------- -------- -------- header
-------- 1111---- -------- -------- -------- -------- remVersion
-------- ----1111 -------- -------- -------- -------- id
-------- -------- 1111---- -------- -------- -------- messageId
-------- -------- ----1111 11111111 -------- -------- period
-------- -------- -------- -------- 11111111 11111111 duration
*/

#ifndef __REM_ROBOT_BUZZER_H
#define __REM_ROBOT_BUZZER_H

#include <stdbool.h>
#include <stdint.h>
#include "REM_BaseTypes.h"

typedef struct _REM_RobotBuzzerPayload {
    uint8_t payload[PACKET_SIZE_REM_ROBOT_BUZZER];
} REM_RobotBuzzerPayload;

typedef struct _REM_RobotBuzzer {
    uint32_t   header              ; // integer [0, 255]             Header byte indicating the type of packet
    uint32_t   remVersion          ; // integer [0, 15]              Version of roboteam_embedded_messages
    uint32_t   id                  ; // integer [0, 15]              Id of the robot
    uint32_t   messageId           ; // integer [0, 15]              Id of the message
    uint32_t   period              ; // integer [0, 4095]            Sound that the buzzer makes.
    float      duration            ; // float   [0.000, 5.000]       Duration of the sound
} REM_RobotBuzzer;

// ================================ GETTERS ================================
static inline uint32_t REM_RobotBuzzer_get_header(REM_RobotBuzzerPayload *remrbp){
    return ((remrbp->payload[0]));
}

static inline uint32_t REM_RobotBuzzer_get_remVersion(REM_RobotBuzzerPayload *remrbp){
    return ((remrbp->payload[1] & 0b11110000) >> 4);
}

static inline uint32_t REM_RobotBuzzer_get_id(REM_RobotBuzzerPayload *remrbp){
    return ((remrbp->payload[1] & 0b00001111));
}

static inline uint32_t REM_RobotBuzzer_get_messageId(REM_RobotBuzzerPayload *remrbp){
    return ((remrbp->payload[2] & 0b11110000) >> 4);
}

static inline uint32_t REM_RobotBuzzer_get_period(REM_RobotBuzzerPayload *remrbp){
    return ((remrbp->payload[2] & 0b00001111) << 8) | ((remrbp->payload[3]));
}

static inline float REM_RobotBuzzer_get_duration(REM_RobotBuzzerPayload *remrbp){
    uint32_t _duration = ((remrbp->payload[4]) << 8) | ((remrbp->payload[5]));
    return (_duration * 0.0000762951094835) + 0.0000000000000000;
}

// ================================ SETTERS ================================
static inline void REM_RobotBuzzer_set_header(REM_RobotBuzzerPayload *remrbp, uint32_t header){
    remrbp->payload[0] = header;
}

static inline void REM_RobotBuzzer_set_remVersion(REM_RobotBuzzerPayload *remrbp, uint32_t remVersion){
    remrbp->payload[1] = ((remVersion << 4) & 0b11110000) | (remrbp->payload[1] & 0b00001111);
}

static inline void REM_RobotBuzzer_set_id(REM_RobotBuzzerPayload *remrbp, uint32_t id){
    remrbp->payload[1] = (id & 0b00001111) | (remrbp->payload[1] & 0b11110000);
}

static inline void REM_RobotBuzzer_set_messageId(REM_RobotBuzzerPayload *remrbp, uint32_t messageId){
    remrbp->payload[2] = ((messageId << 4) & 0b11110000) | (remrbp->payload[2] & 0b00001111);
}

static inline void REM_RobotBuzzer_set_period(REM_RobotBuzzerPayload *remrbp, uint32_t period){
    remrbp->payload[2] = ((period >> 8) & 0b00001111) | (remrbp->payload[2] & 0b11110000);
    remrbp->payload[3] = period;
}

static inline void REM_RobotBuzzer_set_duration(REM_RobotBuzzerPayload *remrbp, float duration){
    uint32_t _duration = (uint32_t)(duration / 0.0000762951094835);
    remrbp->payload[4] = (_duration >> 8);
    remrbp->payload[5] = _duration;
}

// ================================ ENCODE ================================
static inline void encodeREM_RobotBuzzer(REM_RobotBuzzerPayload *remrbp, REM_RobotBuzzer *remrb){
    REM_RobotBuzzer_set_header              (remrbp, remrb->header);
    REM_RobotBuzzer_set_remVersion          (remrbp, remrb->remVersion);
    REM_RobotBuzzer_set_id                  (remrbp, remrb->id);
    REM_RobotBuzzer_set_messageId           (remrbp, remrb->messageId);
    REM_RobotBuzzer_set_period              (remrbp, remrb->period);
    REM_RobotBuzzer_set_duration            (remrbp, remrb->duration);
}

// ================================ DECODE ================================
static inline void decodeREM_RobotBuzzer(REM_RobotBuzzer *remrb, REM_RobotBuzzerPayload *remrbp){
    remrb->header        = REM_RobotBuzzer_get_header(remrbp);
    remrb->remVersion    = REM_RobotBuzzer_get_remVersion(remrbp);
    remrb->id            = REM_RobotBuzzer_get_id(remrbp);
    remrb->messageId     = REM_RobotBuzzer_get_messageId(remrbp);
    remrb->period        = REM_RobotBuzzer_get_period(remrbp);
    remrb->duration      = REM_RobotBuzzer_get_duration(remrbp);
}

#endif /*__REM_ROBOT_BUZZER_H*/
