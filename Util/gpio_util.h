/*
 * gpio_util.h
 *
 *  Created on: 8 April 2019
 *      Author: Cas Doornkamp
 */

#ifndef UTILS_GPIO_UTIL_H_
#define UTILS_GPIO_UTIL_H_

#include "stdbool.h"
#include "stm32f7xx_hal.h"


#define LOW 0
#define HIGH 1

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

// Wheels Locked
extern GPIO_Pin RB_LOCK_pin;
extern GPIO_Pin RF_LOCK_pin;
extern GPIO_Pin LB_LOCK_pin;
extern GPIO_Pin LF_LOCK_pin;

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
extern GPIO_Pin LED0_pin;
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

// MTi
extern GPIO_Pin MTi_RST_pin;
extern GPIO_Pin MTi_IRQ_pin;

// Wireless
extern GPIO_Pin SX_IRQ_pin;
extern GPIO_Pin SX_RST_pin;
extern GPIO_Pin SX_NSS_pin;
extern GPIO_Pin SX_BUSY_pin;


/////////////////////////////////////////////// GPIO UTILITY FUNCTIONS

// Set a GPIO Pin
inline void set_Pin(GPIO_Pin p, bool value)
{
	HAL_GPIO_WritePin(p.PORT, p.PIN, value);
}

// Read a GPIO Pin
inline GPIO_PinState read_Pin(GPIO_Pin p)
{
	return HAL_GPIO_ReadPin(p.PORT, p.PIN);
}

// Toggle a GPIO Pin
inline void toggle_Pin(GPIO_Pin p)
{
	HAL_GPIO_TogglePin(p.PORT, p.PIN);
}

static inline uint16_t get_Id(){
	uint16_t ID = 0;
	ID |= !read_Pin(ID0_pin) <<3;
	ID |= !read_Pin(ID1_pin) <<2;
	ID |= !read_Pin(ID2_pin) <<1;
	ID |= !read_Pin(ID3_pin);
	return ID;
}

#endif /* UTILS_GPIO_UTIL_H_ */
