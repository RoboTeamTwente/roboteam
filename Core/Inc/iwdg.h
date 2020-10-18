/* 
	file implementing the Independent WatchDoG feature of the STM32F7
	"Inspired" by datasheet from ST
*/

#ifndef IWDG_H
#define IWDG_H

#include "stm32f7xx.h"
// timer runs at 32 kHz (with prescaler 4), so 8 kHz. with a counter of 1600 it is a timeout of 200 ms before it resets
#define RELOAD_VALUE 1600

typedef struct
{
	__IO uint32_t Key;   /*!< IWDG Key register,       Address offset: 0x00 */
	__IO uint32_t Pre;   /*!< IWDG Prescaler register, Address offset: 0x04 */
	__IO uint32_t Reload;  /*!< IWDG Reload register,    Address offset: 0x08 */
	__IO uint32_t Status;   /*!< IWDG Status register,    Address offset: 0x0C */
	__IO uint32_t Window; /*!< IWDG Window register,    Address offset: 0x10 */
} IWDG_REG;

typedef struct
{
	IWDG_REG*		mem;
	struct
	{
		uint32_t	Prescaler;
		uint32_t	Reload;
		uint32_t	Window;
	} Settings;

}IWDG_Handle;

// Init function that sets the registers
HAL_StatusTypeDef IWDG_Init(IWDG_Handle* hiwdg);
// This function resets the timer and needs to be called every while loop
void IWDG_Refresh(IWDG_Handle* hiwdg);

#endif // IWDG_H
