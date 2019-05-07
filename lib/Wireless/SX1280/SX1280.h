#ifndef __SX1280_H
#define __SX1280_H

#include "SX1280_Constants.h"
#include "stdbool.h"
#include "stm32f7xx_hal.h"
#include "gpio_util.h"
#include "string.h" // to use memcpy properly

////////////////////////////////////// Structs
typedef struct _SX1280_Settings{
	uint32_t frequency;
	float channel;
	uint8_t packettype;
	uint8_t payloadLength;
	uint8_t variableLength;
	uint8_t txPower;
	uint8_t TX_ramp_time;
	uint8_t syncWordEnable; // not used
	uint32_t syncWords[3];
	uint8_t syncWordTolerance;
	uint8_t syncSensitivity;
	uint8_t TXoffset;
	uint8_t RXoffset;
	uint8_t crcSeed[2];
	uint8_t crcPoly[2];
	uint8_t ModParam[3];
	uint8_t PacketParam[7];
	uint16_t DIOIRQ[4];
	uint8_t periodBase;	// period base for time out (set to 62.5us)
	uint16_t periodBaseCount; // period base count for time out (set to 24 -> 62.5us*24=1.5ms to send packet out)
} SX1280_Settings;

typedef struct _SX1280_Packet_Status{
	uint8_t RFU;		// reserved for future use (does nothing)
	uint8_t RSSISync;	// signal power = -RSSISync/2 (dBm)
	uint8_t errors;		// bit6: SyncError, 5: length error, 4: CRC error, 3: abort error, 2: header received, 1: packet received, 0: CTRL busy
	uint8_t status;		// bit5: rxNoAck (dynamic payload), bit0: packet sent
	uint8_t sync;		// bit 2:0 : 001 syncword1 detected, 010 syncword2, 100 syncword3
} SX1280_Packet_Status;

// Global struct
typedef struct _SX1280{
	SX1280_Settings* SX_settings;		// pointer to struct containing all the settings
	SX1280_Packet_Status* Packet_status;// last known packet status

	uint8_t* TXbuf;			// pointer to transmit buffer
	uint8_t* RXbuf;			// pointer to receive buffer

	uint8_t payloadLength;	// length of received packet
	uint8_t RXbufferoffset;	// start location of received packet
	uint16_t irqStatus;		// last received irq status

	uint8_t SX1280_status;	// last received status of the SX1280
	bool expect_packet;		// bool to specify if a packet is expected in the buffer
	bool new_data;			// new data flag

	SPI_HandleTypeDef* SPI;	// spi used
	bool SPI_used;			// guard to limit one function to use spi

	// wiring
	GPIO_Pin BUSY_pin;
	GPIO_Pin CS_pin;
	GPIO_Pin IRQ_pin;
	GPIO_Pin RST_pin;
} SX1280;

extern uint32_t robot_syncWord[]; // check bottom of this file for the sycnwords

////////////////////////////////////// public functions
void SX1280Setup(SX1280* SX);
void SX1280WakeUp(SX1280* SX);

// runtime functions
bool writeBuffer(SX1280* SX, uint8_t * data, uint8_t Nbytes);
void readBuffer(SX1280* SX, uint8_t Nbytes);
uint8_t getStatus(SX1280* SX);

void setDIOIRQParams(SX1280* SX);
void clearIRQ(SX1280* SX, uint16_t mask);
uint16_t getIRQ(SX1280* SX);
void getPacketStatus(SX1280* SX);
void getRXBufferStatus(SX1280* SX);

////////////////////////////////////// private functions
// private init functions
void setRFFrequency(SX1280* SX, uint32_t frequency);
void setModulationParam(SX1280* SX);
void setBufferBase(SX1280* SX, uint8_t tx_address, uint8_t rx_address);
void setPacketType(SX1280* SX, uint8_t type);
uint8_t getPacketType(SX1280* SX);
void setPacketParam(SX1280* SX);
void setTXParam(SX1280* SX, uint8_t power, uint8_t rampTime); // power 0-31 --> -18 - 13 dBm, ramptime (us)
void setRegulatorMode(SX1280* SX, uint8_t mode); // choose between integrated LDO (uses more power) or external DC-DC converter

void setSyncSensitivity (SX1280* SX, uint8_t syncSensitivity);

bool setSyncWords(SX1280* SX, uint32_t syncWord_1, uint32_t syncWord_2, uint32_t syncWord_3);
void setSyncWordTolerance(SX1280* SX, uint8_t syncWordTolerance);
//bool setSyncWord_1(SX1280* SX, uint32_t word);

void setCrcSeed(SX1280* SX, uint8_t seed_msb, uint8_t seed_lsb);
void setCrcPoly(SX1280* SX, uint8_t poly_msb, uint8_t poly_lsb);

void setChannel(SX1280* SX, float new_channel);

// state functions
void setStandby(SX1280* SX, uint8_t config);
void setSleep(SX1280* SX, uint8_t config);
bool setRX(SX1280* SX, uint8_t base, uint16_t count);
bool setFS(SX1280* SX);
void setAutoFS(SX1280* SX, bool enable);
bool setTX(SX1280* SX, uint8_t base, uint16_t count);
void setAutoTX(SX1280* SX, uint16_t wait_time);

// register specific
void modifyRegister(SX1280* SX, uint16_t address, uint8_t mask, uint8_t set_value);
bool writeRegister(SX1280* SX, uint16_t address, void* data, uint8_t Nbytes);
void readRegister(SX1280* SX, uint16_t address, uint8_t* data, uint8_t Nbytes);

// Send/Receive data
bool SendData(SX1280* SX, uint8_t Nbytes);
bool SendData_DMA(SX1280* SX, uint8_t Nbytes);
void DMA_Callback(SX1280* SX);
#endif // __SX1280_H

// Sync Words for RobotID 'abcde'
/*
abcde	0abc deab cdea bcde		abcd e1ab cdea bcde		hex
00000	0000 0000 0000 0000 	0000 0100 0000 0000		0000 0400
00001	0000 0100 0010 0001		0000 1100 0010 0001		0421 0C21
00010	0000 1000 0100 0010		0001 0100 0100 0010		0842 1442
00011	0000 1100 0110 0011		0001 1100 0110 0011		0CC3 1CC3
00100	0001 0000 1000 0100		0010 0100 1000 0100		1084 2484
00101	0001 0100 1010 0101		0010 1100 1010 0101		14A5 2CA5
00110	0001 1000 1100 0110		0011 0100 1100 0110		18C6 34C6
00111	0001 1100 1110 0111		0011 1100 1110 0111		1CD7 3CD7
01000	0010 0001 0000 1000		0100 0101 0000 1000		2108 4508
01001	0010 0101 0010 1001		0100 1101 0010 1001		2529 4D29
01010	0010 1001 0100 1010		0101 0101 0100 1010		298A 554A
01011	0010 1101 0110 1011		0101 1101 0110 1011		2D6D 5D6D
01100	0011 0001 1000 1100		0110 0101 1000 1100		318C 658C
01101	0011 0101 1010 1101		0110 1101 1010 1101		35AD 6DAD
01110	0011 1001 1100 1110		0111 0101 1100 1110		39CE 75CE
01111	0011 1101 1110 1111		0111 1101 1110 1111		3DEF 7DEF

abcde	0abc deab cdea bcde		abcd e1ab cdea bcde		hex
10000	0100 0010 0001 0000 	1000 0110 0001 0000		8210 8610
10001	0100 0110 0011 0001		1000 1110 0011 0001		8631 8E31
10010	0100 1010 0101 0010		1001 0110 0101 0010		8A52 9652
10011	0100 1110 0111 0011		1001 1110 0111 0011		8ED3 9ED3
10100	0101 0010 1001 0100		1010 0110 1001 0100		9294 A694
10101	0101 0110 1011 0101		1010 1110 1011 0101		96B5 AEB5
10110	0101 1010 1101 0110		1011 0110 1101 0110		9AD6 B6D6
10111	0101 1110 1111 0111		1011 1110 1111 0111		9EE7 BEE7
11000	0110 0011 0001 1000		1100 0111 0001 1000		B318 C718
11001	0110 0111 0011 1001		1100 1111 0011 1001		B739 CF39
11010	0110 1011 0101 1010		1101 0111 0101 1010		BB9A D75A
11011	0110 1111 0111 1011		1101 1111 0111 1011		BF7D DF7D
11100	0111 0011 1001 1100		1110 0111 1001 1100		C39C E79C
11101	0111 0111 1011 1101		1110 1111 1011 1101		C7BD EFBD
11110	0111 1011 1101 1110		1111 0111 1101 1110		CBDE F7DE
11111	0111 1111 1111 1111		1111 1111 1111 1111		CFFF FFFF
*/