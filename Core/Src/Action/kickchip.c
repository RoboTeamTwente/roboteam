#include "gpio.h"
#include <stdbool.h>
#include "tim.h"
#include "kickchip.h"
#include "../PuttyInterface/PuttyInterface.h"

#define Timestep 20

kick_states kick_state = kick_Idle;
bool Charge;
KICK_ports Kick_port 	= { Kick_GPIO_Port	, Kick_Pin	};
KICK_ports Chip_port 	= { Chip_GPIO_Port	, Chip_Pin	};
KICK_ports Charge_port 	= { Charge_GPIO_Port, Charge_Pin};
// utility function for setting pin values
inline void set_pin(KICK_ports p, bool value)
{
	HAL_GPIO_WritePin(p.PORT, p.PIN, value);
}


// Init variables and call kick_Callback(), which will keep calling itself.
void kick_Init(){
	kick_state = kick_Charging;


	set_pin(Kick_port,GPIO_PIN_RESET);		// Kick off
	set_pin(Chip_port, GPIO_PIN_RESET);		// Chip off
	set_pin(Charge_port, GPIO_PIN_SET);		// kick_Charging on
	Charge = false;
	kick_Callback();
}

// deactivates the kicker board to sleep mode
void kick_DeInit(){
	HAL_TIM_Base_Stop(&htim13);
	kick_state = kick_Idle;
	set_pin(Kick_port, GPIO_PIN_RESET);		// Kick off
	set_pin(Chip_port, GPIO_PIN_RESET);		// Chip off
	set_pin(Charge_port, GPIO_PIN_RESET);	// kick_Charging off
}

/*
 *  Initiates the kick_Shoot of the robot at a percentage of the max power.
 *  Will only do this if in state kick_Ready.
 *  Sets a timer for kick_Callback() to stop the kick. The power of the kick depends on the time interval between this function and the callback.
 */
void kick_Shoot(int percentage, bool kick)
{
	if(percentage < 1){
		percentage = 1;
	}else if(percentage > 100){
		percentage = 100;
	}
	if(kick_state == kick_Ready)
	{
		kick_state = kick_Kicking;								// Block kick_Charging
		set_pin(Charge_port, GPIO_PIN_RESET); 						// Disable kick_Charging
		set_pin(kick ? Kick_port : Chip_port, GPIO_PIN_SET); 				// Kick/Chip on
//		uprintf("kick_Kicking\n\r");

		HAL_TIM_Base_Stop(&htim13);								// Stop timer
		__HAL_TIM_CLEAR_IT(&htim13,TIM_IT_UPDATE);
		__HAL_TIM_SET_COUNTER(&htim13, 0);						// Clear timer
		__HAL_TIM_SET_AUTORELOAD(&htim13, percentage * (kick ? 4 : 6));	// Set kick time
		HAL_TIM_Base_Start_IT(&htim13);   						// Start timer
	}
}


/*
 * Handles all the callbacks for kick_Kicking, chipping and kick_Charging
 * This function keeps setting a new timer to call itself.
 */
void kick_Callback()
{
		HAL_TIM_Base_Stop(&htim13);				// Stop the timer
		__HAL_TIM_SET_COUNTER(&htim13, 0);


		int Callback_time = 0;					// local variable for setting timer duration
		if(kick_state == kick_Kicking)
		{
			set_pin(Kick_port, GPIO_PIN_RESET);		// Kick off
			set_pin(Chip_port, GPIO_PIN_RESET);		// Chip off
			kick_state = kick_Charging;
			set_pin(Charge_port, GPIO_PIN_SET);		// Turn kick_Charging on
			Callback_time = 100*Timestep;		// Set timer to 100ms
		}
		else if(kick_state == kick_Charging)
		{
			if(!HAL_GPIO_ReadPin(Charge_done_GPIO_Port, Charge_done_Pin))		// If kick_Charging is done
			{
				set_pin(Charge_port, GPIO_PIN_RESET);								// Turn kick_Charging off
				kick_state = kick_Ready;										// Go to kick_Ready state
				Charge = false;
			}
			Callback_time = 100*Timestep;										// Set timer to 100ms
		}
		else if(kick_state == kick_Ready)
		{
			set_pin(Charge_port, Charge);	// Turn kick_Charging off
			Callback_time = 1000*Timestep;		// Set timer to 100ms
			Charge = !Charge;
		}

		__HAL_TIM_CLEAR_IT(&htim13,TIM_IT_UPDATE);			// Clear timer
		__HAL_TIM_SET_AUTORELOAD(&htim13, Callback_time);	// Set callback time to defined value
		HAL_TIM_Base_Start_IT(&htim13);
}


void kick_Stateprint()
{
//	uprintf("Block = [%d]\n\r", kick_state);
}
