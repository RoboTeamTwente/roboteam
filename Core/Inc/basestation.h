#ifndef __BASESTATION_H
#define __BASESTATION_H

#include <stdbool.h>
#include <stdint.h>
#include "FT812Q.h"

void init();
void loop();

void updateTouchState(TouchState* touchState);
bool handlePacket(uint8_t* Buf, uint32_t *Len);
bool handleStatistics(void);

#endif  /* __BASESTATION_H */