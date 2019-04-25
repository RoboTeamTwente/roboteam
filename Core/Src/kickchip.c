
#include "../Inc/kickchip.h"

//TODO: current set-up would only chip or kick if charged, but does not let know when it does that.

///////////////////////////////////////////////////// STRUCTS

static kick_states kickState = kick_Off;

///////////////////////////////////////////////////// VARIABLES

static bool charged = false;	// true if the capacitor is fully charged
static int power = 0; 			// percentage of maximum kicking power


static int count = 0;
///////////////////////////////////////////////////// PRIVATE FUNCTION DECLARATIONS

// Stops and starts the timer for a certain period of time
void resetTimer(int timePeriod);

///////////////////////////////////////////////////// PUBLIC FUNCTION IMPLEMENTATIONS

void kick_Init(){
	charged = false;
	kickState = kick_Charging;
	set_Pin(Kick_pin,0);		// Kick off
	set_Pin(Chip_pin, 0);		// Chip off
	set_Pin(Charge_pin, 1);		// kick_Charging on
	kick_Callback(); 			// go to callback for the first time
}

void kick_DeInit(){
	charged = false;
	kickState = kick_Off;
	set_Pin(Kick_pin, 0);		// Kick off
	set_Pin(Chip_pin, 0);		// Chip off
	set_Pin(Charge_pin, 0);		// kick_Charging off
	HAL_TIM_Base_Stop(TIM_KC);
}

void kick_Callback()
{
	Putty_printf("kick state: %d \n\r", kickState);

	int callbackTime = 0;
	switch(kickState){
	case kick_Ready:
		charged = !charged;
		set_Pin(Charge_pin, !charged);
		callbackTime = TIMER_FREQ/READY_CALLBACK_FREQ;
		count++;
		break;
	case kick_Charging:
		if(!read_Pin(Charge_done_pin)){
			set_Pin(Charge_pin, 0);
			charged = true;
			kickState = kick_Ready;
		}
		else {
			set_Pin(Kick_pin, 0);
			set_Pin(Chip_pin, 0);
			set_Pin(Charge_pin, 1);
			charged = false;
		}
		callbackTime = TIMER_FREQ/CHARGING_CALLBACK_FREQ;
		break;
	case kick_Kicking:
		set_Pin(Kick_pin, 0);		// Kick off
		set_Pin(Chip_pin, 0);		// Chip off
		kickState = kick_Charging;
		callbackTime = TIMER_FREQ/KICKING_CALLBACK_FREQ;
		Putty_printf("count: %d\n\r", count);
		count = 0;
		break;
	case kick_Off:
		set_Pin(Kick_pin, 0);		// Kick off
		set_Pin(Chip_pin, 0);		// Chip off
		set_Pin(Charge_pin, 0);		// kick_Charging off
		callbackTime = TIMER_FREQ/OFF_CALLBACK_FREQ;
		break;
	}
	resetTimer(callbackTime);
}

kick_states kick_GetState(){
	return kickState;
}

void kick_SetPower(int input){
	if(input < 1){
		power = 1;
	}else if(input > 100){
		power = 100;
	}else {
		power = input;
	}
}

void kick_Shoot(bool doChip)
{
	if(kickState == kick_Ready)
	{
		Putty_printf("kicking! power = %d \n\r", power);
		kickState = kick_Kicking;
		set_Pin(Charge_pin, 0); 								// Disable kick_Charging
		set_Pin(doChip ? Chip_pin : Kick_pin, 1); 				// Kick/Chip on

		resetTimer(power * (doChip ? CHIP_TIME : KICK_TIME));
	}
}

///////////////////////////////////////////////////// PRIVATE FUNCTION IMPLEMENTATIONS

void resetTimer(int timePeriod)
{
	HAL_TIM_Base_Stop(TIM_KC);							// Stop timer
	__HAL_TIM_CLEAR_IT(TIM_KC, TIM_IT_UPDATE);			// Clear timer
	__HAL_TIM_SET_COUNTER(TIM_KC, 0);      				// Reset timer
	__HAL_TIM_SET_AUTORELOAD(TIM_KC, timePeriod);		// Set callback time to defined value
	HAL_TIM_Base_Start_IT(TIM_KC);						// Start timer
}

