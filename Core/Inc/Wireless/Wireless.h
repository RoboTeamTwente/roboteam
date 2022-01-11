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

#define MAX_BUF_LENGTH 128

#define WIRELESS_YELLOW_CHANNELS 0
#define WIRELESS_BLUE_CHANNELS 1

#define WIRELESS_YELLOW_FEEDBACK_CHANNEL -5 // 2.395 GHz
#define WIRELESS_YELLOW_COMMAND_CHANNEL -15 // 2.385 GHz
#define WIRELESS_BLUE_FEEDBACK_CHANNEL -25
#define WIRELESS_BLUE_COMMAND_CHANNEL -35

typedef enum WIRELESS_CHANNEL {
    YELLOW_CHANNEL = 0,
    BLUE_CHANNEL = 1
} WIRELESS_CHANNEL;

SX1280 SX1280_TX_struct;
SX1280 SX1280_RX_struct;
SX1280 * SX_TX; // pointer to the datastruct for SX TX module
SX1280 * SX_RX; // pointer to the datastruct for SX RX module

// Public Functions
SX1280 * Wireless_Init(float channel, SPI_HandleTypeDef * WirelessSpi, uint8_t mode); // mode=0 -> TX, mode=1 -> RX
void SendPacket(SX1280* SX, uint8_t * data, uint8_t Nbytes);
void ReceivePacket(SX1280* SX);
void Wireless_IRQ_Handler(SX1280* SX, uint8_t * data, uint8_t Nbytes);
void Wireless_DMA_Handler(SX1280* SX);

void SX1280_updateChannel(WIRELESS_CHANNEL channel);
WIRELESS_CHANNEL SX1280_getCurrentChannel();

#endif /* WIRELESS_WIRELESS_H_ */
