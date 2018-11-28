/*
 * Geneva.h
 *
 *  Created on: Nov 21, 2018
 *      Author: Cas Doornkamp
 */
/*
Description: controls the kick/chip interface

Instructions:

Extra functions:

GPIO Pins:

Notes:

*/

#ifndef KICK_CHIP_H_
#define KICK_CHIP_H_

#include "gpio_util.h"

#define kicktim htim13
#define KICK true
#define CHIP false

// possible states where the kicker can be in
typedef enum{
	kick_Ready,		// ready for kicking
	kick_Kicking,	// currently kicking/chipping
	kick_Charging,	// charging, it cannot kick at this moment
	kick_Idle		// the kicker circuit is not active
}kick_states;

typedef struct kickchip{
    kick_states Kstate;
    bool geneva_state;
    bool ball_sensor_state;
    bool Charge;
}kickchip;

kickchip kick;

//  set init values for the gpio ports and initiate timer
void kick_Init(kickchip* kick);
//  de-activate kicker circuit
void kick_DeInit(kickchip* kick);
//  shoot or chip the ball, given a certain strength
kick_states kick_Shoot(kickchip* kick, int percentage, bool kick_chip);
//  timer callback function controlling the states
void kick_Callback(kickchip* kick);
//  handle kick command
void Kick_handle_command(kickchip *kick, char* input, int len);
#endif
