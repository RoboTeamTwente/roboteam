#ifndef __BASESTATION_H
#define __BASESTATION_H

#include <stdbool.h>
#include <stdint.h>
#include "FT812Q.h"

// Yes these are the same value, but they have different use cases
// Anything regarding the number of robots will go up to 15 : [0, 15)
// anything regarding robot ids will go up to and including 15 : [0, 15]
#define MAX_NUMBER_OF_ROBOTS 15
#define MAX_ROBOT_ID 15

void init();
void loop();

void updateTouchState(TouchState* touchState);
bool handlePacket(uint8_t* Buf, uint32_t Len);
bool handleStatistics(void);

#endif  /* __BASESTATION_H */