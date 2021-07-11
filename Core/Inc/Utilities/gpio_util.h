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
extern GPIO_Pin SX_TX_IRQ;
extern GPIO_Pin SX_TX_RST;
extern GPIO_Pin SX_TX_BUSY;
extern GPIO_Pin SX_TX_CS;

extern GPIO_Pin SX_RX_IRQ;
extern GPIO_Pin SX_RX_RST;
extern GPIO_Pin SX_RX_BUSY;
extern GPIO_Pin SX_RX_CS;

extern GPIO_Pin LD_3; // stm32 nucleo on-board LED2
extern GPIO_Pin LD_2; // stm32 nucleo on-board LED2

extern GPIO_Pin LD_TX; // basestation TX LED
extern GPIO_Pin LD_RX;
extern GPIO_Pin LD_ACTIVE; // basestation ACTIVE LED
extern GPIO_Pin LD_USB; // basestation USB LED
extern GPIO_Pin LD_LED1; // basestation LED1
extern GPIO_Pin LD_LED2; // basestation LED2 // MATCHES WITH stm32 nucleo on-board LED1
extern GPIO_Pin LD_LED3; // basestation LED3
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

inline uint16_t get_Id(){
	uint16_t ID = 15;
//	ID |= read_pin(ID0_pin) <<3;
//	ID |= read_pin(ID1_pin) <<2;
//	ID |= read_pin(ID2_pin) <<1;
//	ID |= read_pin(ID3_pin);
	return ID;
}

#endif /* UTILS_GPIO_UTIL_H_ */
