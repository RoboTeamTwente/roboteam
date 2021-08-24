
#ifndef __SX1280_CONSTANTS_H
#define __SX1280_CONSTANTS_H

#include "main.h"

#define PLL_STEP (double)0.00504123076	// because compiler doesn't like the version below
//#define PLL_STEP ((double)(2^18)/(52000000))	// step/Hz


#define FEEDBACK_HEADER	0xCCCCCCCC
// ----------------------------------- Generic Codes

// taken from https://www.semtech.com/uploads/documents/DS_SX1280-1_V2.2.pdf
// List of Op-Codes						SEND								RETURN
#define GET_STATUS 			0xC0
#define SX_WRITE_REG 		0x18		// address 2Bytes, data[n]			-
#define SX_READ_REG 		0x19		// address 2Bytes					data[n]
#define WRITE_BUF			0x1A		// offset, data[n]					-
#define READ_BUF			0x1B		// offset							data[n]
#define SET_SLEEP			0x84		// sleep config						-
#define SET_STANDBY			0x80		// standby config					-
#define SET_FS				0xC1		// -								-
#define SET_TX				0x83		// PeriodBase, BaseCount			-
#define SET_RX				0x82		// PeriodBase, BaseCount			-
#define SET_RX_DUTY			0x94		// RXPeriodBase,...,SleepPeriodBase	-
#define SET_CAD				0xC5		// -								-
#define SET_TX_CON			0xD1		// -								-
#define SET_TX_PRE			0xD2		// -								-
#define SET_PACKET			0x8A		// PacketType						-
#define GET_PACKET 			0x03		// -								PacketType
#define SET_RF_F			0x86		// RF Frequency 2Bytes				-
#define SET_TX_PARAM		0x8E		// Power, RampTime					-
#define SET_CAD_PARAM		0x88		// CADSymbolNum						-
#define	SET_BUF_RX_TX		0x8F		// TXBaseAddress, RXBaseAddress		-
#define SET_MOD_PARAM		0x8B		// ModParam1, .. , ModParam3		-
#define SET_PACKET_PARAM	0x8C		// PacketParam1, .., PacketParam7	-
#define GET_RX_BUF_STATUS	0x17		// -								PayloadLength, RXBufOffset
#define GET_PACKET_STATUS	0x1D		// -								PacketStatus[39:0]
#define	SET_DIO_IRQ_PARAM	0x8D		// IRQMask 2B, DIO1Mask 2B, ..		-
#define GET_IRQ_STATUS		0x15		// -								irqStatus 2B
#define	CLR_IRQ_STATUS		0x97		// IRQMask 2Bytes					-

#define SET_AUTO_FS			0x9E		// 1:enable, 0:disable				-
#define SET_AUTO_TX			0x98		// time	after receive				-
#define SET_PERF_COUNTER	0x9C		// Perf Counter Mode				-

#define SET_REGULATOR_MODE	0x96		// Regulator mode (0 only LDO)

// ----------------------------------- FLRC specific codes

// ModParam 1 (Bit-Rate & Band-Width)		Code	// BR (Mb/s)	BW (MHz)
#define FLRC_BR_1_300_BW_1_2 				0x45	// 1.3			1.2
#define FLRC_BR_1_000_BW_1_2				0x69	// 1.04			1.2
#define FLRC_BR_0_650_BW_0_6				0x86	// 0.65			0.6
#define FLRC_BR_0_520_BW_0_6				0xAA	// 0.52			0.6
#define FLRC_BR_0_325_BW_0_3				0xC7	// 0.325		0.3
#define FLRC_BR_0_260_BW_0_3				0xEB	// 0.26			0.3

// ModParam 2 (Coding-Rate)
#define FLRC_CR_1_2		0x00	// 1/2
#define FLRC_CR_3_4		0x02	// 3/4
#define FLRC_CR_1_0		0x04	// 1

// ModParam 3 (filtering)
#define BT_DIS			0x00	// No Filtering
#define BT_1			0x10	// 1
#define BT_0_5			0x20	// 0.5

// PacketType
#define PACKET_TYPE_GFSK 	0x00
#define PACKET_TYPE_LORA 	0x01
#define PACKET_TYPE_RANGING 0x02
#define PACKET_TYPE_FLRC	0x03
#define PACKET_TYPE_BLE		0x04

// PacketParam 1 (AGC Preamble length)
#define PREAMBLE_LENGTH_4_BITS			0x00
#define PREAMBLE_LENGTH_8_BITS			0x10
#define PREAMBLE_LENGTH_12_BITS			0x20
#define PREAMBLE_LENGTH_16_BITS			0x30
#define	PREAMBLE_LENGTH_20_BITS			0x40
#define PREAMBLE_LENGTH_24_BITS			0x50
#define PREAMBLE_LENGTH_28_BITS			0x60
#define PREAMBLE_LENGTH_32_BITS			0x70

// PacketParam 2 (Sync Word Length)
#define FLRC_SYNC_NOSYNC		0x00	// 21-bits preamble
#define FLRC_SYNC_WORD_LEN_P32S	0x04	// 21-bits preamble + 32 bits Sync word

// PacketParam 3 (Sync Word combination)
#define RX_DISABLE_SYNC_WORD 		0x00	// no
#define RX_MATCH_SYNC_WORD_1		0x10	// 1
#define RX_MATCH_SYNC_WORD_2		0x20	// 2
#define RX_MATCH_SYNC_WORD_1_2		0x30	// 1 or 2
#define RX_MATCH_SYNC_WORD_3		0x40	// 3
#define RX_MATCH_SYNC_WORD_1_3		0x50	// 1 or 3
#define RX_MATCH_SYNC_WORD_2_3		0x60	// 2 or 3
#define RX_MATCH_SYNC_WORD_1_2_3	0x70	// 1 or 2 or 3

// PacketParam 4 (fixed/Variable Packet)
#define PACKET_FIXED_LENGTH 		0x00
#define PACKET_VARIABLE_LENGTH	 	0x20

// PacketParam 5 (Payload Length)

// PacketParam 6 (CRC Length)
#define CRC_OFF 	0x00
#define CRC_1_BYTE 	0x10
#define CRC_2_BYTE	0x20
#define CRC_3_BYTE	0x30

// PacketParam 7 (Whitening)
#define NO_WHITENING	0x08	// disables whitening (whitening cannot be used with FLRC)

// Ramp Time					// ramp time (us)
#define RADIO_RAMP_02_US 0x00 	// 2
#define RADIO_RAMP_04_US 0x20 	// 4
#define RADIO_RAMP_06_US 0x40 	// 6
#define RADIO_RAMP_08_US 0x60 	// 8
#define RADIO_RAMP_10_US 0x80 	// 10
#define RADIO_RAMP_12_US 0xA0 	// 12
#define RADIO_RAMP_16_US 0xC0 	// 16
#define RADIO_RAMP_20_US 0xE0 	// 20

// ----------------------------------- Addresses
#define CRC_POLY_MSB	0x09C6
#define CRC_POLY_LSB	0x09C7
#define CRC_INIT_MSB	0x09C8
#define CRC_INIT_LSB	0x09C9

#define SYNCWORD1		0x09CF 	// (MSB) up for LSB 32bits long (REVERSE!)
#define SYNCWORD2		0x09D4 	// ...
#define SYNCWORD3		0x09D9	// ...

#define SYNC_SENS		0x0891	// set bits 7:6 (0xC0) of 0x0891 to 0x3 for high sensitivity
#define SYNC_TOL		0x09CD	// set sync word tolerance

// ----------------------------------- Period Base
#define BASE_4000_us	0x03
#define BASE_1000_us	0x02
#define BASE_62_us		0x01
#define BASE_15_us		0x00

// different types of IRQ that can be set
typedef enum _SX1280_IRQ{
	NONE 				= 0,
	TX_DONE 			= 1<<0,		// ALL
	RX_DONE 			= 1<<1,		// ALL
	SYNCWORD_VALID 		= 1<<2,		// GFSK/BLE/FLRC
	SYNCWORD_ERROR		= 1<<3,		// FLRC
	CRC_ERROR			= 1<<6,		// GFSK/BLE/FLRC/LoRa
	RXTX_TIMEOUT		= 1<<14,	// ALL
	PREAMBLE_DETECTED	= 1<<15,	// ALL (if SetLongPreamble is set)
	ALL 				= 0xFFFF	// ALL (utility to clear all interupts)
} SX1280_IRQ;

#endif //__SX1280_CONSTANTS_H
