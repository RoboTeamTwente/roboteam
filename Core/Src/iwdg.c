
#include "iwdg.h"
#include "stm32f7xx.h"

#define TIMEOUT		48u		// wait for max 48 ms before starting

#define START		0x0000CCCC
#define RELOAD		0x0000AAAA
#define SET_WRITE	0x00005555
#define LOCK_WRITE	0x00000000


HAL_StatusTypeDef IWDG_Init(IWDG_Handle* hiwdg)
{
	hiwdg->mem = (IWDG_REG*)IWDG;
	hiwdg->Settings.Prescaler = 0x00000000;	// precaler 4
	hiwdg->Settings.Window = RELOAD_VALUE;
	hiwdg->Settings.Reload = RELOAD_VALUE;


	WRITE_REG((hiwdg)->mem->Key, 0x0000CCCCu);	// start IWDG
	WRITE_REG((hiwdg)->mem->Key, 0x00005555u);	// set write access

	hiwdg->mem->Pre = hiwdg->Settings.Prescaler;
	hiwdg->mem->Reload = hiwdg->Settings.Reload;

	

	/* Wait for register to be updated */
	uint32_t  tickstart = HAL_GetTick();
	while (hiwdg->mem->Status != RESET){
		if ((HAL_GetTick() - tickstart) > TIMEOUT){
			return HAL_TIMEOUT;
		}
	}

	hiwdg->mem->Window = hiwdg->Settings.Window;
	return HAL_OK;
}

void IWDG_Refresh(IWDG_Handle* hiwdg)
{
	WRITE_REG((hiwdg)->mem->Key, RELOAD);
}
