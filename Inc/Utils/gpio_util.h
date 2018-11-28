/*
 * gpio_util.h
 *
 *  Created on: 26 okt. 2018
 *      Author: Cas Doornkamp
 */

#ifndef UTILS_GPIO_UTIL_H_
#define UTILS_GPIO_UTIL_H_

#include "gpio.h"
#include "stdbool.h"

// abstract a GPIO pin combination to a struct
typedef struct GPIO_Pin{
	GPIO_TypeDef * PORT;
	uint16_t PIN;
} GPIO_Pin;

/////////////////////////////////// LIST OF KNOWN GPIO PINS

// Kick/Chip
extern GPIO_Pin Kick_pin;
extern GPIO_Pin Chip_pin;
extern GPIO_Pin Charge_pin;
extern GPIO_Pin Charge_done_pin;

// Geneva
extern GPIO_Pin Geneva_PWM_pin;
extern GPIO_Pin Geneva_DIR_A_pin;
extern GPIO_Pin Geneva_DIR_B_pin;
extern GPIO_Pin Geneva_ENC_A_pin;
extern GPIO_Pin Geneva_ENC_B_pin;

// Wheels PWM
extern GPIO_Pin RB_PWM_pin;
extern GPIO_Pin RF_PWM_pin;
extern GPIO_Pin LB_PWM_pin;
extern GPIO_Pin LF_PWM_pin;

// Wheels DIR
extern GPIO_Pin RB_DIR_pin;
extern GPIO_Pin RF_DIR_pin;
extern GPIO_Pin LB_DIR_pin;
extern GPIO_Pin LF_DIR_pin;

// Wheels ENC
extern GPIO_Pin RB_ENC_A_pin;
extern GPIO_Pin RF_ENC_A_pin;
extern GPIO_Pin LB_ENC_A_pin;
extern GPIO_Pin LF_ENC_A_pin;
extern GPIO_Pin RB_ENC_B_pin;
extern GPIO_Pin RF_ENC_B_pin;
extern GPIO_Pin LB_ENC_B_pin;
extern GPIO_Pin LF_ENC_B_pin;

// Battery
extern GPIO_Pin Bat_pin;

// Dribbler
extern GPIO_Pin Dribbler_PWM_pin;

// LEDs
extern GPIO_Pin LED1_pin;
extern GPIO_Pin LED2_pin;
extern GPIO_Pin LED3_pin;
extern GPIO_Pin LED4_pin;
extern GPIO_Pin LED5_pin;
extern GPIO_Pin LED6_pin;

// ID select
extern GPIO_Pin ID0_pin;
extern GPIO_Pin ID1_pin;
extern GPIO_Pin ID2_pin;
extern GPIO_Pin ID3_pin;

// Frequency select
extern GPIO_Pin FRQ_sel;

// Xsens 
GPIO_Pin Xsens_enable_pin			= {XSENS_nRST_GPIO_Port		, XSENS_nRST_Pin	};

/////////////////////////////////////////////// GPIO UTILITY FUNCTIONS

// Set a GPIO Pin
inline void set_pin(GPIO_Pin p, bool value)
{
	HAL_GPIO_WritePin(p.PORT, p.PIN, value);
}

// Read a GPIO Pin
inline GPIO_PinState read_pin(GPIO_Pin p)
{
	return HAL_GPIO_ReadPin(p.PORT, p.PIN);
}

// Toggle a GPIO Pin
inline void toggle_pin(GPIO_Pin p)
{
	HAL_GPIO_TogglePin(p.PORT, p.PIN);
}


#endif /* UTILS_GPIO_UTIL_H_ */
