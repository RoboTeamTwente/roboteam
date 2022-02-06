#ifndef __SX1280_H
#define __SX1280_H

#include "SX1280_Constants.h"
#include "SX1280_Packet_Definitions.h"
#include "stdbool.h"
#include "stm32f7xx_hal.h"
#include "gpio_util.h"
#include "string.h" // to use memcpy properly

////////////////////////////////////// Enums

typedef enum _SX1280_Error{
	SX1280_OK,
	SX1280_BUSY,
	SX1280_PARAM_ERROR,
	SX1280_TIMEOUT,
	SX1280_FAIL,
} SX1280_Error;

////////////////////////////////////// Structs
typedef struct _SX1280_Settings{
	uint8_t packettype;
	SX1280_ModulationParam ModParam;
	SX1280_PacketParam PacketParam;
	uint8_t TXoffset;
	uint8_t RXoffset;
	uint8_t TXpower;
	SX1280_RampTime TXrampTime;
	uint8_t syncWordTolerance; // Amount of bits that may be wrong in a received syncword to still pass the syncword valid check (4 bits)
	uint8_t syncSensitivity;
	uint16_t crcSeed;
	uint16_t crcPoly;
	uint16_t DIOIRQ[4];
	SX1280_PeriodBase periodBase;	// period base for time out (set to 62.5us)
	uint16_t periodBaseCount; // period base count for time out (set to 24 -> 62.5us*24=1.5ms to send packet out)
} SX1280_Settings;


typedef struct _SX1280Interface{
	 GPIO_Pin BusyPin;
	 GPIO_Pin Reset;
	 GPIO_Pin CS;
	 SPI_HandleTypeDef* SPI;
	volatile bool active_transfer;
	volatile uint8_t* TXbuf;
	volatile uint8_t* RXbuf;
	volatile SX1280_Status SX1280_status;
	void* logger; // TODO: Implement logger feedback
}SX1280_Interface;

////////////////////////////////////// public functions
void SX1280WakeUp(SX1280_Interface* interface);

// ---------- Section 11.3 - Get Status, page 72
SX1280_Status getStatus(SX1280_Interface* SX);

// ---------- Section 11.4 - Register Access, page 74
bool writeRegister(SX1280_Interface* SX, uint16_t address, void* data, uint8_t Nbytes);
void readRegister(SX1280_Interface* SX, uint16_t address, uint8_t* data, uint8_t Nbytes);

// Helper Function
void modifyRegister(SX1280_Interface* SX, uint16_t address, uint8_t mask, uint8_t set_value);

// ---------- Section 11.5 - Data Buffer Operations, page 75
void writeBuffer(SX1280_Interface* interface, uint8_t TXoffset, uint8_t * data, uint8_t Nbytes);
void writeBuffer_DMA(SX1280_Interface* interface, uint8_t TXoffset, uint8_t * data, uint8_t Nbytes);
void readBuffer(SX1280_Interface* interface, uint8_t RXoffset, uint8_t* dest, uint8_t Nbytes);
void readBuffer_DMA(SX1280_Interface* interface, uint8_t RXoffset, uint8_t* dest, uint8_t Nbytes);

// ---------- Section 11.6 - Radio Operation Modes, page 77
void setSleep(SX1280_Interface* SX, uint8_t config);
void setStandby(SX1280_Interface* SX, uint8_t config);
bool setFS(SX1280_Interface* SX);
bool setTX(SX1280_Interface* SX, uint8_t base, uint16_t count);
bool setRX(SX1280_Interface* SX, uint8_t base, uint16_t count);
void setAutoTX(SX1280_Interface* SX, uint16_t wait_time);
void setAutoFS(SX1280_Interface* SX, bool enable);

// ---------- Section 11.7 - Radio Configuration, page 85
void setPacketType(SX1280_Interface* SX, SX1280_PacketType type);
uint8_t getPacketType(SX1280_Interface* SX);
void setRFFrequency(SX1280_Interface* SX, float frequency);
void setTXParam(SX1280_Interface* SX, uint8_t power, SX1280_RampTime rampTime); // power 0-31 --> -18 - 13 dBm, ramptime (us)
void setBufferBase(SX1280_Interface* SX, uint8_t tx_address, uint8_t rx_address);
void setModulationParam(SX1280_Interface* SX, SX1280_ModulationParam* param);
void setPacketParam(SX1280_Interface* SX, SX1280_PacketParam* param);

// Helper Function
void setChannel(SX1280_Interface* SX, float new_channel);

// ---------- Section 11.8 - Communication Status Information, page 92
void getRXBufferStatus(SX1280_Interface* SX, SX1280_RX_Buffer_Status* bufferStatus);
void getPacketStatus(SX1280_Interface* SX, SX1280_Packet_Status* status);

// ---------- Section 11.9 IRQ Handling, page 95
void setDIOIRQParams(SX1280_Interface* SX, uint16_t DIOIRQ[4]);
void getIRQ(SX1280_Interface* SX, uint16_t* irq);
void clearIRQ(SX1280_Interface* SX, uint16_t mask);

// ---------- Register Settings
void setRegulatorMode(SX1280_Interface* SX, uint8_t mode); // choose between integrated LDO (uses more power) or external DC-DC converter

/* 4.2.1 Low Power Mode and High Sensitivity Mode, page 30 */
void setSyncSensitivity (SX1280_Interface* SX, uint8_t syncSensitivity);

SX1280_Error setSyncWord(SX1280_Interface* interface, uint8_t index, uint32_t syncWord);
void setSyncWordTolerance(SX1280_Interface* SX, uint8_t syncWordTolerance);

void setCrcSeed(SX1280_Interface* SX, uint16_t seed);
void setCrcPoly(SX1280_Interface* SX, uint16_t poly);

// ---------- Callback Functions
void DMA_Callback(SX1280_Interface* SX, uint8_t* dest, uint8_t Nbytes);

////////////////////////////////////// private functions
// Helper functions to Send/Receive data over SPI
SX1280_Error SendData(SX1280_Interface* interface, uint8_t Nbytes);
SX1280_Error SendData_DMA(SX1280_Interface* SX, uint8_t Nbytes);

#endif // __SX1280_H