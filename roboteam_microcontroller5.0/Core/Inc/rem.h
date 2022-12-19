#ifndef __REM_H
#define __REM_H

#include "stm32f7xx_hal.h"

void REM_UARTinit(UART_HandleTypeDef *huart);
void REM_UARTCallback(UART_HandleTypeDef *huart);

#endif /* __REM_H */