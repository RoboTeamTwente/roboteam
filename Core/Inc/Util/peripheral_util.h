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
#define MTi_SPI 	(&hspi1)	// MTi SPI
#define COMM_SPI 	(&hspi4)	// wireless SPI
#define BS_I2C 		(&hi2c1)	// Ball Sensor I2C
#define SC_SPI 		(&hqspi)	// Screen QuadSPI


extern SPI_HandleTypeDef hspi1;
extern SPI_HandleTypeDef hspi4;
extern I2C_HandleTypeDef hi2c1;
extern QSPI_HandleTypeDef hqspi;
extern UART_HandleTypeDef huart5;



#endif /* PERIPHERAL_UTIL_H_ */
