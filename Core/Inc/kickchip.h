
/* Description: Makes the robot chip or kick
 *
 * Instructions:
 * 1) First the robot's needs to charge
 * 2) when that has happend the kick or chip command can be executed
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
	kick_Ready,
	kick_Charging,
	kick_Kicking,
	kick_Off
}kick_states;

///////////////////////////////////////////////////// PUBLIC FUNCTION DECLARATIONS

void kick_Init();

void kick_DeInit();

void kick_Callback();

void kick_Shoot(bool kick);

kick_states kick_GetState();

void kick_SetPower(int input);

#endif /* __kickchip_H */
