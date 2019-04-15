
/* Description:
 *
 * Instructions:
 * 1)
 *
 * Extra functions:
 *
 * Notes:
 *
*/

#ifndef __kickchip_H
#define __kickchip_H

#include "../Util/gpio_util.h"
#include "../Util/tim_util.h"

///////////////////////////////////////////////////// STRUCTS

typedef enum{
	On,
	charge,
	kick,
	chip,
	Off
}kick_states;

///////////////////////////////////////////////////// PUBLIC FUNCTION DECLARATIONS

void kick_Init();

void kick_DeInit();

void kick_Callback();

void kick_Shoot();

void kick_Chip();

void kick_SetPer(int input);

#endif /* __kickchip_H */
