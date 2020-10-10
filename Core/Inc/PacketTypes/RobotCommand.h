#ifndef __ROBOT_COMMAND_H
#define __ROBOT_COMMAND_H

#include <stdbool.h>
#include <stdint.h>

#include "BaseTypes.h"

typedef union _robotCommand{
    uint8_t payload[10];
    struct __attribute__((__packed__)){     // Description                      Encoded range               Unit        Value Range
        PACKET_TYPE header          :8;     // packet type                      PACKET_TYPE_ROBOT_COMMAND   -           -
        uint8_t     id              :8;     // destination robot ID             [0, 15]                     -           [0-15]
        uint16_t    rho             :11;    // reference velocity amplitude     [0, 2047]                   0.004m/s    [0, 8.188]
        int16_t     theta           :11;    // reference velocity angle         [-1024, 1023]               0.00307rad  [-pi, pi>
        int16_t     angularVelocity :10;    // reference rotation speed         [-512, 511]                 0.098rad/s  [-16pi, 16pi>
        uint8_t     power           :8;     // Kick/Chip power                  [0, 255]                    0.39%       [0, 100]%
        bool        doKick          :1;     // Kick (Ballsensor)                [0, 1]                      -           {false, true}
        bool        doChip          :1;     // Chip (Ballsensor)                [0, 1]                      -           {false, true}
        bool        force           :1;     // Force a Kick/Chip                [0, 1]                      -           {false, true}
        bool        debugInfo       :1;     // Request debug info (deprecated)  [0, 1]                      -           {false, true}
        bool        useCamInfo      :1;     // Use cam info for alignment       [0, 1]                      -           {false, true}
        uint8_t     geneva          :3;     // geneva angles (deprecated)       [0,7]                       -           [-2, 2]
        uint8_t     dribblerSpeed   :5;     // dribbler rotation speed          [0, 31]                     3.125%      [0, 100]%
        int16_t     camRotation     :11;    // Orientation seen by camera       [-1024, 1023]               0.00307rad  [-pi, pi>
    };
} robotCommand;

// ----------------------------------------------------- Test Code to see how the data is packed  -----------------------------------------------------

// #define BYTE_TO_BIN_PAT "%c%c%c%c%c%c%c%c "
// #define BYTE_TO_BIN(byte) \
//     (byte & 0x01 ? '1' : '0'), \
//     (byte & 0x02 ? '1' : '0'), \
//     (byte & 0x04 ? '1' : '0'), \
//     (byte & 0x08 ? '1' : '0'), \
//     (byte & 0x10 ? '1' : '0'), \
//     (byte & 0x20 ? '1' : '0'), \
//     (byte & 0x40 ? '1' : '0'), \
//     (byte & 0x80 ? '1' : '0')

// robotCommand rc = {0};
// void printPayload(void){
//     for(int i=0; i<9; i++){
//         printf(BYTE_TO_BIN_PAT, BYTE_TO_BIN(rc.payload[i]));
//     }
//     printf("\n");
// }

// rc.header = 0b01100110;
// printPayload();

// rc.header = 0;
// rc.id = 15;
// printPayload();

// rc.id = 0;
// rc.rho = 2047;
// printPayload();

// rc.rho = 0;
// rc.theta = -1;
// printPayload();

// rc.theta = 0;
// rc.angularVelocity = -1;
// printPayload();

// rc.angularVelocity = 0;
// rc.power = 255;
// printPayload();

// rc.power = 0;
// rc.doKick = true;
// printPayload();

// rc.doKick = false;
// rc.doChip = true;
// printPayload();

// rc.doChip = false;
// rc.force = true;
// printPayload();

// rc.force = false;
// rc.debugInfo = true;
// printPayload();

// rc.debugInfo = false;
// rc.useCamInfo = true;
// printPayload();

// rc.useCamInfo = false;
// rc.geneva = -1;
// printPayload();

// rc.geneva = 0;
// rc.dribblerSpeed = 31;
// printPayload();

// rc.dribblerSpeed = 0;
// rc.camRotation = -1;
// printPayload();

#endif /*__ROBOT_COMMAND_H*/