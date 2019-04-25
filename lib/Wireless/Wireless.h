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
#include "SX1280/SX1280_Constants.h"
#include "SX1280/SX1280.h"
#include "main.h"

#define MAX_BUF_LENGTH 128
#define AUTO_TX_TIME 120 // (us)
#define RECEIVEPKTLEN 13 //amount of bytes for a packet sent to the robot

SX1280 SX1280_struct;
SX1280 * SX; // pointer to the datastruct
uint8_t * Bot_to_PC; // pointer to feedback data struct
uint8_t * PC_to_Bot; // pointer to received data struct

// Public Functions
SX1280 * Wireless_Init(float channel, SPI_HandleTypeDef * WirelessSpi);
void SendPacket(SX1280* SX, uint8_t * data, uint8_t Nbytes);
void ReceivePacket(SX1280* SX);
void Wireless_IRQ_Handler(SX1280* SX, uint8_t * data, uint8_t Nbytes);
void Wireless_DMA_Handler(SX1280* SX, uint8_t * output);

#endif /* WIRELESS_WIRELESS_H_ */
