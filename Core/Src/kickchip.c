
#include "../Inc/kickchip.h"
#include <stdbool.h>

//TODO: Check if this works
//TODO: current set-up would only chip or kick if charged, but does not let know when it does that.

///////////////////////////////////////////////////// STRUCTS

kick_states kickState = Off;

///////////////////////////////////////////////////// VARIABLES

static bool charged = false;
static int percentage = 0;

///////////////////////////////////////////////////// PUBLIC FUNCTION IMPLEMENTATIONS

void kick_Init(){
	charged = false;
	kickState = charge;
	set_Pin(Kick_pin,0);		// Kick off
	set_Pin(Chip_pin, 0);		// Chip off
	set_Pin(Charge_pin, 1);		// kick_Charging on
	HAL_TIM_Base_Start(TIM_KC);
}

void kick_DeInit(){
	kickState = Off;
	charged = false;
	set_Pin(Kick_pin, 0);		// Kick off
	set_Pin(Chip_pin, 0);		// Chip off
	set_Pin(Charge_pin, 0);	// kick_Charging off
	HAL_TIM_Base_Stop(TIM_KC);
}

//TODO: why does kickState behave so weird
void kick_Callback()
{
	switch(kickState){
	case On:
		break;
	case charge:
		if(read_Pin(Charge_done_pin)){
			set_Pin(Charge_pin, 0);
			charged = true;
			kickState = On;
		}
		else {
			set_Pin(Kick_pin, 0);
			set_Pin(Chip_pin, 0);
			set_Pin(Charge_pin, 1);
			charged = false;
		}
		break;
	case kick:
		if (charged){
			kick_Shoot();
		} else {
			kickState = charge;
		}
		break;
	case chip:
		if (charged){
			kick_Chip();
		} else {
			kickState = charge;
		}
		break;
	case Off:
		set_Pin(Kick_pin, 0);		// Kick off
		set_Pin(Chip_pin, 0);		// Chip off
		set_Pin(Charge_pin, 0);	// kick_Charging off
		break;
	}
}


void kick_Shoot()
{
	if(percentage < 1){
		percentage = 1;
	}else if(percentage > 100){
		percentage = 100;
	}
	if(charged) {
		set_Pin(Kick_pin, 1);
	} else {
		kickState = charge;
		charged = false;
	}
}

void kick_Chip()
{
	if(percentage < 1){
		percentage = 1;
	}else if(percentage > 100){
		percentage = 100;
	}
	if(charged) {
		set_Pin(Chip_pin, 1);
	} else {
		kickState = charge;
		charged = false;
	}
}

void kick_SetState(kick_states input){
	kickState = input;
}

kick_states kick_GetState(){
	return kickState;
}

void kick_SetPer(int input){
	percentage = input;
}

