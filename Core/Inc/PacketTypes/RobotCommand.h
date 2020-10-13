#ifndef __ROBOT_COMMAND_H
#define __ROBOT_COMMAND_H

#include <stdbool.h>
#include <stdint.h>

#include "BaseTypes.h"

typedef struct _RobotCommandPayload {
    uint8_t payload[PACKET_SIZE_ROBOT_COMMAND];
} RobotCommandPayload;

typedef struct _RobotCommand{
    uint8_t header;
    uint8_t id;
    uint16_t rho;
    int16_t theta;
    int16_t angularVelocity;
    uint8_t power;
    bool doKick;
    bool doChip;
    bool force;
    bool debugInfo;
    bool useCamInfo;
    uint8_t geneva;
    uint8_t dribblerSpeed;
    int16_t camRotation;
} RobotCommand;

inline uint8_t RobotCommand_getHeader(RobotCommandPayload *rcp){
    return rcp->payload[0];
};
inline uint8_t RobotCommand_getId(RobotCommandPayload *rcp){
    return rcp->payload[1];
};
inline uint16_t RobotCommand_getRho(RobotCommandPayload *rcp){
    return ((rcp->payload[2] & 0b11111111) << 3)
         | ((rcp->payload[3] & 0b11100000) >> 5);
};
inline int16_t RobotCommand_getTheta(RobotCommandPayload *rcp){
    return ((rcp->payload[3] & 0b00011111) << 6)
         | ((rcp->payload[4] & 0b11111100) >> 2);
};
inline int16_t RobotCommand_getAngularVelocity(RobotCommandPayload *rcp){
    return ((rcp->payload[4] & 0b00000011) << 8)
         | ((rcp->payload[5] & 0b11111111) >> 0);
};
inline uint8_t RobotCommand_getPower(RobotCommandPayload *rcp){
    return rcp->payload[6];
};
inline bool RobotCommand_getDoKick(RobotCommandPayload *rcp){
    return rcp->payload[7] & 0b10000000;
};
inline bool RobotCommand_getDoChip(RobotCommandPayload *rcp){
    return rcp->payload[7] & 0b01000000;
};
inline bool RobotCommand_getForce(RobotCommandPayload *rcp){
    return rcp->payload[7] & 0b00100000;
};
inline bool RobotCommand_getDebugInfo(RobotCommandPayload *rcp){
    return rcp->payload[7] & 0b00010000;
};
inline bool RobotCommand_getUseCamInfo(RobotCommandPayload *rcp){
    return rcp->payload[7] & 0b00001000;
};
inline uint8_t RobotCommand_getGeneva(RobotCommandPayload *rcp){
    return rcp->payload[7] & 0b00000111;
};
inline uint8_t RobotCommand_getDribblerSpeed(RobotCommandPayload *rcp){
    return (rcp->payload[8] & 0b11111000) >> 3;
};
inline int16_t RobotCommand_getCamRotation(RobotCommandPayload *rcp){
    return ((rcp->payload[8] & 0b00000111) << 8)
         | ((rcp->payload[9] & 0b11111111) >> 0);
};

static inline void fillRobotCommand(RobotCommand *command, RobotCommandPayload *rcp){
    command->header = RobotCommand_getHeader(rcp);
    command->id = RobotCommand_getId(rcp);
    command->rho = RobotCommand_getRho(rcp);
    command->theta = RobotCommand_getTheta(rcp);
    command->angularVelocity = RobotCommand_getAngularVelocity(rcp);
    command->power = RobotCommand_getPower(rcp);
    command->doKick = RobotCommand_getDoKick(rcp);
    command->doChip = RobotCommand_getDoChip(rcp);
    command->force = RobotCommand_getForce(rcp);
    command->debugInfo = RobotCommand_getDebugInfo(rcp);
    command->useCamInfo = RobotCommand_getUseCamInfo(rcp);
    command->geneva = RobotCommand_getGeneva(rcp);
    command->dribblerSpeed = RobotCommand_getDribblerSpeed(rcp);
    command->camRotation = RobotCommand_getCamRotation(rcp);
}

#endif /*__ROBOT_COMMAND_H*/