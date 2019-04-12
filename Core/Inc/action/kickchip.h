
#ifndef __kickchip_H
#define __kickchip_H

#include "gpio.h"

#define KICK true
#define CHIP false
// possible states where the kicker can be 
typedef enum{
	kick_Ready,		// ready for kicking
	kick_Kicking,	// currently kicking/chipping
	kick_Charging,	// charging, it cannot kick at this moment
	kick_Idle		// the kicker circuit is not active
}kick_states;

// kick and chip ports
typedef struct KICK_ports{
	GPIO_TypeDef * PORT;
	uint16_t PIN;
} KICK_ports;

// set init values for the gpio ports and initiate timer
void kick_Init();
// de-activate kicker circuit
void kick_DeInit();
// shoot or chip the ball, given a certain strength
void kick_Shoot(int percentage, bool kick);
//  timer callback function controlling the states
void kick_Callback();
// utility function for printing the state
void kick_Stateprint();
#endif /* __kickchip_H */
