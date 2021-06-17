/*
 * MTi_protocol.h
 *
 *  Created on: 25 april 2019
 *      Author: Cas Doornkamp
 */

#ifndef MTI_PROTOCOL_H_
#define MTI_PROTOCOL_H_

#include <stdlib.h>

/* Page 13. 2.7.5 SPI. The first byte transmitted by the Master is the opcode which identifies what kind of data is 
transmitted by the Master and what kind of data the Master wants to read from the Slave (See MTSSP). The second- to 
fourth byte transmitted are the fill words. These fill words are needed to give the Slave some time to prepare the 
remainder of the transfer. */
#define HEADER_LENGTH 4		// amount of bytes before actual data is sent/received

// OP-codes
typedef enum MTi_opcodes {
	ReadProtocol		= 0x01,		// (read)	current protocol setting
	SetProtocol			= 0x02,		// (write)	change protocol
	SetPipe				= 0x03,		// (write)	send messages (reduced Xbus message)
	PipeStatus			= 0x04,		// (read)	status of notification/measurement pipe
	ReadNotification	= 0x05,		// (read)	read notifications (reduced Xbus message)
	ReadMeasurement		= 0x06		// (read)	read measurement data (reduced Xbus message)
}MTi_opcodes;

// drdy bit values
typedef enum drdyConf {
	Pol				= 0x01,		// 0: idle low		1: idle high
	Output			= 0x02,		// 0: push/pull		1: Open drain
	Notification	= 0x04,		// 0: disable		1: enable
	Measurement		= 0x08		// 0: disable		1: enable
}drdyConf;

// protocol structs
typedef struct Rprotocol {
	uint8_t version;			// version  (1 byte)
	uint8_t drdyconfig;			// drdy config (see drdyConf)
}Rprotocol;

typedef struct PipeStat {
	uint16_t notification_size;	// length of notidications (in bytes)
	uint16_t measurement_size;	// length of measurements  (in bytes)
}PipeStat;

#endif /* MTI_PROTOCOL_H_ */
