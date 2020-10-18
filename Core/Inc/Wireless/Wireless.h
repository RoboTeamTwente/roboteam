/*
 * Wireless.h
 *
 *  Created on: 6 feb. 2019
 *      Author: Cas Doornkamp
 * 
 * Implements functionality for main to communicate with the SX1280
 */

#ifndef WIRELESS_WIRELESS_H_
#define WIRELESS_WIRELESS_H_

#include <stdbool.h>
#include "SX1280_Constants.h"
#include "SX1280.h"
#include "main.h"
#include "packing.h"

#define MAX_BUF_LENGTH 128
#define AUTO_TX_TIME 120 // (us)

#define FEEDBACK_CHANNEL -5	// 2.395 GHz
#define COMMAND_CHANNEL -15 // 2.385 GHz

///////////////////////////////////////////////////// PUBLIC FUNCTION DECLARATIONS

SX1280 * Wireless_Init(float channel, SPI_HandleTypeDef * WirelessSpi);
void Wireless_DeInit();
void SendPacket(SX1280* SX, uint8_t * data, uint8_t Nbytes);
void ReceivePacket(SX1280* SX);
void Wireless_IRQ_Handler(SX1280* SX, uint8_t * data, uint8_t Nbytes);
void Wireless_DMA_Handler(SX1280* SX, uint8_t output[]);
bool checkWirelessConnection();

#endif /* WIRELESS_WIRELESS_H_ */
