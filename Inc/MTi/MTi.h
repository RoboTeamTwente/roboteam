/*
 * MTi.h
 *
 *  Created on: 26 okt. 2018
 *      Author: Cas Doornkamp
 */
/*
Description: acts as the interface between the MTi and the robot and sets up the MTi
	Chip: MTi-3-8A7G6T

Instructions:
	Set things to measure in the data_configurations array (MTi.c)
	Call the init function
	MTi_UART_RxCpltCallback to the uart callback
	add MTi_update to a timer or loop it in main
Extra functions:

GPIO Pins:
	Xsens_enable_pin
Notes:
	acceleration and rotation are stored in the struct

*/
#ifndef MTI_MTI_H_
#define MTI_MTI_H_

// Includes
#include <stdlib.h>
#include "main.h"
#include "Utils/gpio_util.h"
#include "PuTTY.h"
#include "usart.h"
#include "stdint.h"
#include "xbus/xbusparser.h"

// Defines
#define MAX_RAW_MESSAGE_SIZE 2055	// maximum extended message length
#define huartMTi huart2				// which huart interface to use
#define MAX_XDI_CONFIGS 64 			// maximum amount of configuration command that can be send
#define MT_DEBUG 0 					// Set to true if MTi should print debug messages

// Error enum returned by every function
typedef enum Xsens_Status{
	Xsens_OK,
	Xsens_Failed_Init,
	Xsens_Failed_Config,
	Xsens_Failed_DeInit,
	Xsens_Failed_Update,
	Xsens_Failed_Decode,
	Xsens_Failed_Receive,
	Xsens_Failed_Debug
}Xsens_Status;

// Xsens state enum
typedef enum Xsens_state{
	Xsens_Reset,
	Xsens_Config,
	Xsens_Wait_For_Ack,
	Xsens_Measure,
	Xsens_Unknown
}Xsens_state;

// Receive state enum
typedef enum MTi_receive_state{
	unitialized,
	Idle,
	Xsens_receive_5,
	Xsens_receive_rest
}MTi_receive_state;

// data to read and frequencies and storage
typedef struct data_tuple{
	enum XsDataIdentifier ID;
	uint16_t frequency;
	void* data;				// pointer to the data where it needs to be stored
} MTi_data_tuple;


///////////////////////////////////////////////////// MAIN DATA STRUCT
typedef struct MTi_data_struct{
	uint8_t aRxBuffer[MAX_RAW_MESSAGE_SIZE];		// receive buffer for the uart interface
	MTi_data_tuple* data_configurations;			// pointer to an array of things to measure
	uint8_t configuration_total;					// counter for keeping the total of data configurations
	// UART
	uint8_t RxCpltCallback_flag;					// flag set when a new packet has been received over uart 
	uint8_t raw[128];								// transmit buffer of uart
	uint8_t HAL_UART_ErrorCallback_flag;			// flag set if uart had an error
	// XBUS	
	struct XbusParser * XBParser;					// struct for managing the packets between the MTi
	uint8_t cplt_mess_stored_flag;					// When XBP_Handlemessage is called, this value is set true
	struct XbusMessage* ReceivedMessageStorage;		// encoded packet pointer (size of MAX_RAW_MESSAGE_SIZE) allocated in MT_init
	uint32_t MT_Data_succerr[2];					// stores the amount of times it succeeded and failed
	// data
	float angles[3];								// euler angles received 
	float acc[3];									// acceleration of the robot on all axes
	uint16_t packetcounter;							// stores the packet ID
	// states
	bool started_icc;								// bool to define if the in-run compas calibration is running
	uint32_t statusword;							// get state/status of the MTi
	Xsens_state Xstate;								// holds the current MTi state
	MTi_receive_state RX_state;						// holds the current message receiving state
}MTi_data;

struct MTi_data_struct MTi;


///////////////////////////////////////////////////// PUBLIC FUNCTION DEFINITIONS

// calibrate_time : time that the robot will stand still to calibrate
// filter_type : defined in XsFilterProfile
// XFP_General, XFP_High_mag_dep, XFP_Dynamic
// XFP_North_referenced, XFP_VRU_general
Xsens_Status MTi_Init(MTi_data* MTi, uint16_t calibrate_time, enum XsFilterProfile filter_type);
Xsens_Status MTi_DeInit(MTi_data* MTi);
Xsens_Status MTi_Update(MTi_data* MTi);
Xsens_Status MTi_Reset(MTi_data* MTi);
Xsens_Status MTi_UART_RxCpltCallback(MTi_data* MTi);

#endif /* MTI_MTI_H_ */
