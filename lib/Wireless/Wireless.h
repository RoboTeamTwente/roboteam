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
#include "packing.h"
#include "SX1280/SX1280.h"

#define WirelessSpi &hspi2
#define MAX_BUF_LENGTH 128
#define AUTO_TX_TIME 120 // (us)

SX1280* SX; // pointer to the datastruct
feedbackData* Bot_to_PC; // pointer to feedback data struct
receivedData* PC_to_Bot; // pointer to received data struct

// Public Functions
void Wireless_Init(SX1280* SX, uint8_t channel, uint8_t RobotID);
void SendPacket(SX1280* SX, feedbackData* data, uint8_t Nbytes);
void ReceivePacket(SX1280* SX);
void Wireless_IRQ_Handler(SX1280* SX, feedbackData* data, uint8_t Nbytes);
void Wireless_DMA_Handler(SX1280* SX, receivedData* output);


#endif /* WIRELESS_WIRELESS_H_ */
