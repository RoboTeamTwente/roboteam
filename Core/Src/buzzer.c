#include "../Inc/buzzer.h"
#include "../Util/tim_util.h"
#include "buzzer_Tunes.h"

uint32_t buzzer_Duration = 0;

song_struct* song;

///////////////////////////////////////////////////// PUBLIC FUNCTIONS IMPLEMENTATIONS

void buzzer_Init() {
	HAL_TIM_Base_Stop(PWM_Buzzer.TIM);
	buzzer_Duration = 0;
	// play a sound to inform that we are aliiiiiiiiive
	buzzer_Play_Startup();
}

void buzzer_DeInit() {
	HAL_TIM_PWM_Stop(PWM_Buzzer.TIM, PWM_Buzzer.Channel);
	HAL_TIM_Base_Stop_IT(PWM_Buzzer.TIM);
}

void buzzer_Callback() {
	if(song){
		if(buzzer_Duration != 0){
			buzzer_Duration--;
		} else {
			song++;
			HAL_TIM_PWM_Stop(PWM_Buzzer.TIM, PWM_Buzzer.Channel);
			HAL_TIM_Base_Stop_IT(PWM_Buzzer.TIM);
			if(song->period == 0xFFFF){
				song = NULL;
				return;
			}
			buzzer_Play(song);
		}
	}
}

void buzzer_SetPWM_Duty(uint16_t duty) {
	set_PWM(PWM_Buzzer, duty);
}

void buzzer_SetPWM_Period(uint16_t period) {
	// HAL library macro to set timer PWM period in runtime
	__HAL_TIM_SET_AUTORELOAD(PWM_Buzzer.TIM, period);
}

void buzzer_Play(song_struct* tone) {
	tone->period = (tone->period == 0) ? 0xFFFF : ((1e6 / tone->period)-1);
	buzzer_Duration = (0.9e6 / tone->period) * (tone->duration);

	if (tone->period == 0xFFFF) {
		HAL_TIM_Base_Start_IT(PWM_Buzzer.TIM);
		buzzer_SetPWM_Period(tone->period);
		buzzer_SetPWM_Duty(tone->period/2);
	}
	else {
		// set the period and duty cycle
		buzzer_SetPWM_Period(tone->period);
		buzzer_SetPWM_Duty(tone->period/2);
		HAL_TIM_Base_Start_IT(PWM_Buzzer.TIM);
		HAL_TIM_PWM_Start(PWM_Buzzer.TIM, PWM_Buzzer.Channel);
	}
}

void buzzer_Play_Startup(){
	song = startup_song;
	buzzer_Play(song);
}

void buzzer_Play_Tetris(){
	song = tetris;
	buzzer_Play(song);
}

void buzzer_Play_Mario(){
	song = mario_victory;
	buzzer_Play(song);
}
void buzzer_Play_PowerUp(){
	song = powerUp;
	buzzer_Play(song);
}

void buzzer_Play_WarningOne() {
	song = warningOne;
	buzzer_Play(song);
}

void buzzer_Play_WarningTwo() {
	song = warningTwo;
	buzzer_Play(song);
}

void buzzer_Play_WarningThree() {
	song = warningThree;
	buzzer_Play(song);
}

void buzzer_Play_WarningFour() {
	song = warningFour;
	buzzer_Play(song);
}

void buzzer_Play_BridgeBattle() {
	song = bridgeBattle;
	buzzer_Play(song);
}

void buzzer_Play_ImperialMarch() {
	song = imperialMarch;
	buzzer_Play(song);
}

void buzzer_Play_Flatline() {
	song = flatLine;
	buzzer_Play(song);
}
