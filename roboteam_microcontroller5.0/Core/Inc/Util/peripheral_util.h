/*
 * peripheral_util.h
 *
 *  Created on: 9 apr. 2019
 *      Author: cjdoo
 */

#ifndef PERIPHERAL_UTIL_H_
#define PERIPHERAL_UTIL_H_

#include "stm32f7xx_hal.h"

#define UART_PC 	(&huart5)	// PC communication
#define UART_BACK 	(&huart4)	// UART going to backboard
#define MTi_SPI 	(&hspi1)	// MTi SPI
#define COMM_SPI 	(&hspi4)	// wireless SPI
#define BS_I2C 		(&hi2c2)	// Ball Sensor I2C
#define BATT_I2C    (&hi2c1)	// Batt monitor I2C
#define SD          (&hsd1)     // SD SDMMC interface



extern SPI_HandleTypeDef hspi1;
extern SPI_HandleTypeDef hspi4;
extern I2C_HandleTypeDef hi2c1;
extern I2C_HandleTypeDef hi2c2;
extern UART_HandleTypeDef huart5;
extern UART_HandleTypeDef huart4;
extern SD_HandleTypeDef hsd1;



#endif /* PERIPHERAL_UTIL_H_ */
