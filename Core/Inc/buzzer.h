#ifndef INC_BUZZER_H_
#define INC_BUZZER_H_

#include "../Util/control_util.h"
#include "../Util/gpio_util.h"
#include "../Util/tim_util.h"
#include "buzzer_Tunes.h"

///////////////////////////////////////////////////// BUZZER LIMITERS
#define BUZZER_SCALER 4000
#define MAX_BUZZER_DURATION 1000000

///////////////////////////////////////////////////// PUBLIC VARIABLE DECLARATIONS
extern uint32_t buzzer_Duration;

///////////////////////////////////////////////////// PUBLIC FUNCTION DECLARATIONS
void buzzer_Init();

void buzzer_DeInit();

void buzzer_Callback();

void buzzer_SetPWM_Duty(uint16_t duty);

// add functions to play songs here, call in other files where needed
void buzzer_Play_Startup();
void buzzer_Play_Tetris();
void buzzer_Play_Mario();
void buzzer_Play_PowerUp();

///////////////////////////////////////////////////// PRIVATE FUNCTION DECLARATIONS
void buzzer_SetPWM_Period(uint16_t period);

void buzzer_Play(song_struct* tone);


#endif /* INC_BUZZER_H_ */
