#ifndef __BASESTATION_H
#define __BASESTATION_H

#include <stdbool.h>
#include <stdint.h>

void init();
void loop();

bool handlePacket(uint8_t* Buf, uint32_t *Len);

#endif  /* __BASESTATION_H */