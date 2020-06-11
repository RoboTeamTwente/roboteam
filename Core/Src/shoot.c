
#include <shoot.h>

///////////////////////////////////////////////////// STRUCTS

static shoot_states shootState = shoot_Off;

///////////////////////////////////////////////////// VARIABLES

static bool charged = false;	// true if the capacitor is fully charged
static int power = 100; 			// percentage of maximum shooting power

///////////////////////////////////////////////////// PRIVATE FUNCTION DECLARATIONS

// Stops and starts the timer for a certain period of time
void resetTimer(int timePeriod);

int calculateShootingTime(shoot_types type);

///////////////////////////////////////////////////// PUBLIC FUNCTION IMPLEMENTATIONS

void shoot_Init(){
	charged = false;
	shootState = shoot_Charging;
	set_Pin(Kick_pin,0);		// Kick off
	set_Pin(Chip_pin, 0);		// Chip off
	set_Pin(Charge_pin, 1);		// shoot_Charging on
	shoot_Callback(); 			// go to callback for the first time
}

void shoot_DeInit(){
	charged = false;
	shootState = shoot_Off;
	set_Pin(Kick_pin, 0);		// Kick off
	set_Pin(Chip_pin, 0);		// Chip off
	set_Pin(Charge_pin, 0);		// shoot_Charging off
	HAL_TIM_Base_Stop(TIM_SHOOT);
}

void shoot_Callback()
{
	int callbackTime = 0;
	static int count = 0;

	switch(shootState){
	case shoot_Ready:
		set_Pin(Charge_pin, 1); // Keep charging
		callbackTime = TIMER_FREQ/READY_CALLBACK_FREQ;
		break;
	case shoot_Charging:
		if (count >= 5) {
			charged = true;
			count = 0;
			shootState = shoot_Ready;
		}
		else {
			set_Pin(Kick_pin, 0);
			set_Pin(Chip_pin, 0);
			set_Pin(Charge_pin, 1);
			charged = false;
			count++;
		}
		callbackTime = TIMER_FREQ/CHARGING_CALLBACK_FREQ;
		break;
	case shoot_Shooting:
		set_Pin(Kick_pin, 0);		// Kick off
		set_Pin(Chip_pin, 0);		// Chip off
		shootState = shoot_Charging;
		callbackTime = TIMER_FREQ/SHOOTING_CALLBACK_FREQ;
		break;
	case shoot_Off:
		set_Pin(Kick_pin, 0);		// Kick off
		set_Pin(Chip_pin, 0);		// Chip off
		set_Pin(Charge_pin, 0);		// kick_Charging off
		callbackTime = TIMER_FREQ/OFF_CALLBACK_FREQ;
		break;
	}
	resetTimer(callbackTime);
}

shoot_states shoot_GetState(){
	return shootState;
}

void shoot_SetPower(int input){
	if(input < 1){
		power = 1;
	}else if(input > 100){
		power = 100;
	}else {
		power = input;
	}
}

void shoot_Shoot(shoot_types type)
{
	bool genevaNotTurning = (geneva_GetPWM() == 0);
	if(shootState == shoot_Ready && genevaNotTurning)
	{
//		Putty_printf("shooting! power = %d \n\r", power);
		shootState = shoot_Shooting;
		set_Pin(Charge_pin, 0); 								// Disable shoot_Charging
		set_Pin(type == shoot_Kick ? Kick_pin : Chip_pin, 1); 				// Kick/Chip on

		resetTimer(calculateShootingTime(type));
	}
}

///////////////////////////////////////////////////// PRIVATE FUNCTION IMPLEMENTATIONS

void resetTimer(int timePeriod)
{
	HAL_TIM_Base_Stop(TIM_SHOOT);							// Stop timer
	__HAL_TIM_CLEAR_IT(TIM_SHOOT, TIM_IT_UPDATE);			// Clear timer
	__HAL_TIM_SET_COUNTER(TIM_SHOOT, 0);      				// Reset timer
	__HAL_TIM_SET_AUTORELOAD(TIM_SHOOT, timePeriod);		// Set callback time to defined value
	HAL_TIM_Base_Start_IT(TIM_SHOOT);						// Start timer
}

int calculateShootingTime(shoot_types type) {
	if (type == shoot_Kick) {
		return ((1.0 - power/100.0) * MIN_KICK_TIME + (power/100.0) * MAX_KICK_TIME);
	} else if (type == shoot_Chip) {
		return ((1.0 - power/100.0) * MIN_CHIP_TIME + (power/100.0) * MAX_CHIP_TIME);
	}
	return 0;
}
