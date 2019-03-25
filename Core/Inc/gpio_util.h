/*
 * gpio_util.h
 *
 *  Created on: 26 okt. 2018
 *      Author: Cas Doornkamp
 */

#ifndef UTILS_GPIO_UTIL_H_
#define UTILS_GPIO_UTIL_H_

#include "main.h"
#include "stdbool.h"

#define HIGH 1
#define LOW 0

// abstract a GPIO pin combination to a struct
typedef struct GPIO_Pin{
	GPIO_TypeDef * PORT;
	uint16_t PIN;
} GPIO_Pin;

/////////////////////////////////// LIST OF KNOWN GPIO PINS
extern GPIO_Pin SX_IRQ;
extern GPIO_Pin SX_RST;
extern GPIO_Pin SX_BUSY;
extern GPIO_Pin LD_3;
extern GPIO_Pin LD_2;

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
