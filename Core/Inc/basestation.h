#ifndef __BASESTATION_H
#define __BASESTATION_H

#include <stdbool.h>
#include <stdint.h>
#include "FT812Q.h"

#define MAX_NUMBER_OF_ROBOTS 16
#define MAX_ROBOT_ID (MAX_NUMBER_OF_ROBOTS-1)

void init();
void loop();

void updateTouchState(TouchState* touchState);
bool handlePacket(uint8_t* packet, uint32_t packet_length);

#endif  /* __BASESTATION_H */