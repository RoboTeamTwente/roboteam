
#ifndef __SX1280_CONSTANTS_H
#define __SX1280_CONSTANTS_H

#include "main.h"


/* 4.4 PLL, page 30 */
// Frf(steps) = 2^18(steps)/Fosc(Hz) * rfFreq(Hz)
#define XTAL_FREQ ((double)52000000.0)
#define STEPS ((double)262144.0)
#define PLL_STEP_TO_HZ (XTAL_FREQ/STEPS)
#define PLL_HZ_TO_STEP (STEPS/XTAL_FREQ)

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

// ----------------------------------- SX1280 Radio Configuration

// PacketType
/* Table 11-42: PacketType Definition, page 86 */
typedef enum{
	PACKET_TYPE_GFSK	= 0x00,
	PACKET_TYPE_LORA	= 0x01,
	PACKET_TYPE_RANGING	= 0x02,
	PACKET_TYPE_FLRC	= 0x03,
	PACKET_TYPE_BLE		= 0x04,
} SX1280_PacketType;


// MODULATION PARAM SETTINGS: 11.7.7 SetModulationParams, page 89

/* Table 6-4: Valid FLRC Data Rate and Bandwidth Combinations, page 38 */
// ModParam 1 (Bit-Rate & Band-Width)		Code	// BR (Mb/s)	BW (MHz)
typedef enum{
	FLRC_BR_1_000_BW_1_2 =					0x69,	// 1.04			1.2
	FLRC_BR_1_300_BW_1_2 =					0x45,	// 1.3			1.2
	FLRC_BR_0_650_BW_0_6 =					0x86,	// 0.65			0.6
	FLRC_BR_0_520_BW_0_6 =					0xAA,	// 0.52			0.6
	FLRC_BR_0_325_BW_0_3 =					0xC7,	// 0.325		0.3
	FLRC_BR_0_260_BW_0_3 =					0xEB,	// 0.26			0.3
} SX1280_BitRate;

/* Table 6-5: Effective FLRC Data Rates Based upon FEC Usage with Resulting Sensitivities, page 39 */
// ModParam 2 (Coding-Rate)
typedef enum{
	FLRC_CR_1_2 = 		0x00,	// 1/2
	FLRC_CR_3_4 = 		0x02,	// 3/4
	FLRC_CR_1_0 = 		0x04,	// 1
} SX1280_CodingRate;

/* Gaussian Filtering, page 40 */
// ModParam 3 Gaussian Filtering
typedef enum{
	BT_DIS	= 0x00,	// No Filtering
	BT_1	= 0x10,	// 1
	BT_0_5	= 0x20,	// 0.5
} SX1280_GaussFilter;


// PACKET PARAM SETTINGS: 7.3 FLRC Packet, page 47

/* Table 14-34: AGC Preamble Length Definition in FLRC Packet, page 122 */
// PacketParam 1 (AGC Preamble length)
typedef enum{
	PREAMBLE_LENGTH_4_BITS	= 0x00,
	PREAMBLE_LENGTH_8_BITS	= 0x10,
	PREAMBLE_LENGTH_12_BITS	= 0x20,
	PREAMBLE_LENGTH_16_BITS	= 0x30,
	PREAMBLE_LENGTH_20_BITS	= 0x40,
	PREAMBLE_LENGTH_24_BITS	= 0x50,
	PREAMBLE_LENGTH_28_BITS	= 0x60,
	PREAMBLE_LENGTH_32_BITS	= 0x70,
} SX1280_PreambleLength;

/* Table 14-35: Sync Word Length Definition in FLRC Packet, page 123 */
// PacketParam 2 (Sync Word Length)
typedef enum{
	FLRC_SYNC_NOSYNC		= 0x00,	// 21-bits preamble
	FLRC_SYNC_WORD_LEN_P32S	= 0x04,	// 21-bits preamble + 4 bytes for Sync word of which 31 bits are available
} SX1280_SyncWordEnable;

/* Table 14-36: Sync Word Combination in FLRC Packet, page 123 */
// PacketParam 3 Sync words to look for
typedef enum{
	RX_DISABLE_SYNC_WORD 		= 0x00,	// no
	RX_MATCH_SYNC_WORD_1		= 0x10,	// 1
	RX_MATCH_SYNC_WORD_2		= 0x20,	// 2
	RX_MATCH_SYNC_WORD_1_2		= 0x30,	// 1 or 2
	RX_MATCH_SYNC_WORD_3		= 0x40,	// 3
	RX_MATCH_SYNC_WORD_1_3		= 0x50,	// 1 or 3
	RX_MATCH_SYNC_WORD_2_3		= 0x60,	// 2 or 3
	RX_MATCH_SYNC_WORD_1_2_3	= 0x70,	// 1 or 2 or 3
} SX1280_MatchSyncWord;

/* Table 14-37: Packet Length Type Definition in FLRC Packet, page 124 */
// PacketParam 4 (fixed/Variable Packet)
typedef enum{
	PACKET_FIXED_LENGTH 	= 0x00,
	PACKET_VARIABLE_LENGTH	= 0x20,
} SX1280_PacketLengthType;

/* Table 14-38: Payload Length Definition in FLRC Packet, page 124 */
// PacketParam 5 (Payload Length)
#define MIN_PAYLOAD_SIZE 0x06
#define MAX_PAYLOAD_SIZE 0x7F

/* Table 14-39: CRC Definition in FLRC Packet, page 124 */
// PacketParam 6 (CRC Length)
typedef enum{
	CRC_OFF 	= 0x00,
	CRC_1_BYTE 	= 0x10,
	CRC_2_BYTE	= 0x20,
	CRC_3_BYTE	= 0x30,
} SX1280_CRCLength;

// PacketParam 7 (Whitening)
#define NO_WHITENING	0x08	// disables whitening (whitening cannot be used with FLRC)

// Ramp Time					// ramp time (us)
typedef enum{
	RADIO_RAMP_02_US = 0x00, 	// 2
	RADIO_RAMP_04_US = 0x20, 	// 4
	RADIO_RAMP_06_US = 0x40, 	// 6
	RADIO_RAMP_08_US = 0x60, 	// 8
	RADIO_RAMP_10_US = 0x80, 	// 10
	RADIO_RAMP_12_US = 0xA0, 	// 12
	RADIO_RAMP_16_US = 0xC0, 	// 16
	RADIO_RAMP_20_US = 0xE0, 	// 20
} SX1280_RampTime;

// ----------------------------------- Period Base
typedef enum{
	BASE_4000_us	= 0x03,
	BASE_1000_us	= 0x02,
	BASE_62_us		= 0x01,
	BASE_15_us		= 0x00,
} SX1280_PeriodBase;

// ----------------------------------- Addresses
#define CRC_POLY_MSB	0x09C6
#define CRC_POLY_LSB	0x09C7
#define CRC_INIT_MSB	0x09C8
#define CRC_INIT_LSB	0x09C9

#define SYNCWORD1		0x09CF 	// starting (MSB) address for 32bits syncword (REVERSE!)
#define SYNCWORD2		0x09D4 	// ...
#define SYNCWORD3		0x09D9	// ...

#define SYNC_SENS		0x0891	// set bits 7:6 (0xC0) of 0x0891 to 0x3 for high sensitivity
#define SYNC_TOL		0x09CD	// sync word tolerance register, only bits 0:3 are valid

// Different types of IRQ that can be set. Only interrupts triggerable in FLRC mode implemented.
/* 11.9 IRQ Handling - Table 11-73: IRQ Register, page 95 */
typedef enum{
	IRQ_NONE 				= 0,
	IRQ_TX_DONE 			= 1<<0,		// ALL
	IRQ_RX_DONE 			= 1<<1,		// ALL
	IRQ_SYNCWORD_VALID 		= 1<<2,		// GFSK/BLE/FLRC
	IRQ_SYNCWORD_ERROR		= 1<<3,		// FLRC
	IRQ_CRC_ERROR			= 1<<6,		// GFSK/BLE/FLRC/LoRa
	IRQ_RXTX_TIMEOUT		= 1<<14,	// ALL
	IRQ_PREAMBLE_DETECTED	= 1<<15,	// ALL (if SetLongPreamble is set)
	IRQ_ALL 				= 0xFFFF	// ALL (utility to clear all interupts)
} SX1280_IRQ;

/* 11.3 GetStatus Command - Table 11-5: Status Byte Definition - bit 4:2, page 73 */
typedef enum{
	Reserved = 0,
	Success = 1,        /* Transceiver has successfully processed the command */
	DataAvailable = 2,  /* A packet has succesfully been received and can be retrieved by the host */
	TimeOut = 3,        /* Host command time-out, a command from the host took too long. The command has to be resend */
	ProcessingErr = 4,  /* Command processing error, Host command was incorrect, either the lenth did not match, or the content was wrong */
	Failed = 5,         /* Failure to execute command, The command was succesfully processed, but the SX could not execute the command */
	TxDone = 6          /* Transmission of the current packet has completed */
} SX1280_CommandStatus;

/* 11.3 GetStatus Command - Table 11-5: Status Byte Definition - bit 7:5, page 73 */
typedef enum{
	Reserved0 = 0,
	Reserved1 = 1,
	STBY_RC = 2,
	STBY_XOSC = 3,
	FS = 4,
	RX = 5,
	TX = 6
} SX1280_State;

/* 11.8.2 GetPacketStatus - Table 11-68: Error Packet Status Byte, page 94 */
// bit6: SyncError, 5: length error, 4: CRC error, 3: abort error, 2: header received, 1: packet received, 0: CTRL busy
typedef enum{
	SX1280_CTRL_BUSY 		= 1 << 0, // Packet Controller busy (RX/TX)
	SX1280_PACKET_RECIEVED = 1 << 1, // a packet has been received, might be invalid (RX)
	SX1280_HEADER_RECEIVED = 1 << 2, // Header received (dynamic length packets, RX)
	SX1280_ABORT_ERROR 	= 1 << 3, // current packet was aborted (RX/TX)
	SX1280_CRC_ERROR		= 1 << 4, // Received packet has a crc mismatch (RX, crc enabled)
	SX1280_LENGTH_ERROR	= 1 << 5, // received packet length is greater than the payload length specified in packet param
	SX1280_SYNC_ERROR		= 1 << 6, // Sync address detected (RX, syncword enabled)
} SX1280_Packet_Errors;

/* 11.8.2 GetPacketStatus - Table 11-68: Error Packet Status Byte, page 94 */
typedef enum{
	SYNCWORD_ERROR = 0b000,
	SYNCWORD_1 = 0b001,
	SYNCWORD_2 = 0b010,
	SYNCWORD_3 = 0b100,
} SX1280_Packet_Syncword;

#endif //__SX1280_CONSTANTS_H
