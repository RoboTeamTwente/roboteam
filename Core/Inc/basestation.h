#ifndef __BASESTATION_H
#define __BASESTATION_H

#include <stdbool.h>
#include <stdint.h>
#include "FT812Q.h"
#include "REM_BaseTypes.h"
#include "REM_BasestationConfiguration.h"

#define MAX_NUMBER_OF_ROBOTS 16
#define MAX_ROBOT_ID (MAX_NUMBER_OF_ROBOTS-1)

void init();
void loop();

bool handleREM_BasestationGetConfiguration();
bool handleREM_BasestationConfiguration(REM_BasestationConfigurationPayload* payload);
void updateTouchState(TouchState* touchState);
bool handlePackets(uint8_t* packets_buffer, uint32_t packets_buffer_length);

#endif  /* __BASESTATION_H */