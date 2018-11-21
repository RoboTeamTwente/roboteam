/*
 * MTi.c
 *
 *  Created on: 26 okt. 2018
 *      Author: Cas Doornkamp
 */

#include "MTi.h"
#include <string.h>
#include "xbusutility.h"
#include "xsdeviceid.h"
#include "gpio_util.h"

#define TIME_OUT 1000U

///////////////////////////////////////////////////// PRIVATE FUNCTION DECLERATIONS
// Xbus functions
static void XBP_handleMessage(struct XbusMessage const* message);
static void* XBP_allocateBuffer(size_t bufSize);
static void XBP_deallocateBuffer(void const* buffer);

// Uart interface handlers
static Xsens_Status MTi_BuildConfig(MTi_data* MTi, enum XsDataIdentifier XDI, uint16_t frequency, bool complete);
static void SendXbusMessage(MTi_data* MTi, struct XbusMessage XbusMessage);
static Xsens_Status processAck(MTi_data* MTi, enum XsMessageId XMID);
static inline void ErrorHandler(struct XbusMessage const* message);
static Xsens_Status Decode_Packet(MTi_data* MTi);

//calibration functions
static Xsens_Status MTi_NoRotation(MTi_data* MTi, uint16_t seconds);
static Xsens_Status MTi_SetFilterProfile(MTi_data* MTi, uint8_t filter);
static Xsens_Status MTi_UseIcc(MTi_data* MTi);

// state change functions
static Xsens_Status MTi_GoToConfig(MTi_data* MTi);
static Xsens_Status MTi_GoToMeasure(MTi_data* MTi);

///////////////////////////////////////////////////// PUBLIC FUNCTION IMPLEMENTATIONS

Xsens_Status MTi_Init(MTi_data* MTi, uint16_t calibrate_time, enum XsFilterProfile filter_type){
	// init state for UART
	MTi->RX_state = Xsens_receive_5;
	MTi->cplt_mess_stored_flag = 0;
	MTi->ReceivedMessageStorage = malloc(MAX_RAW_MESSAGE_SIZE);    // Reserve memory to store the longest possible message
	
    // set the MTi in reset state
    set_pin(Xsens_enable_pin, 0);
	
    // Create a structure to contain the callback functions
    struct XbusParserCallback XBP_callback = {						
		.handleMessage = XBP_handleMessage,
		.allocateBuffer = XBP_allocateBuffer,
		.deallocateBuffer = XBP_deallocateBuffer
	};
	MTi->XBParser = XbusParser_create(&XBP_callback);
    if(MTi->XBParser == NULL){
		return Xsens_Failed_Init;
	}

    // start MTi operation of specific sensors and their update frequencies
	set_pin(Xsens_enable_pin, 1);
	while(MTi->Xstate != Xsens_Config)  HAL_Delay(10); // Wait till MTi is ready
	uprintf("started MTi Operation\n\r");

	// set measurement configurations
	MTi->data_configurations = data_configurations;
	MTi->configuration_total = sizeof data_configurations / sizeof *data_configurations;

	for(int i = 0; i < MTi->configuration_total-1; i++){
		MTi_data_tuple conf = MTi->data_configurations[i];
		bool last = i == (MTi->configuration_total-1);
		MTi_BuildConfig(MTi, conf.ID, conf.frequency, last);
	}
	
	// set filter profile defined in XsFilterProfile
	// 0 = general, 1 = high magnetic dep, 2 = dynamic
	// 3 = assumes stable magnetic field, 4 = high dynamic magnetic field (unreferenced heading)
	MTi_SetFilterProfile(MTi, filter_type);

	// set MTi to measure state
	if(MTi_GoToMeasure(MTi)) return Xsens_Failed_Init;
	HAL_Delay(50);
	// calibrate rotation sensor for STAND_STILL seconds
	if(MTi_NoRotation(MTi,calibrate_time)) return Xsens_Failed_Init;
	//MT_UseIcc();
	return Xsens_OK;
}

Xsens_Status MTi_DeInit(MTi_data* MTi){
	MTi->RX_state = unitialized;
	if(HAL_UART_Abort(&huartMTi) != HAL_OK){
		MTi->Xstate = Xsens_Unknown;
		return Xsens_Failed_DeInit;
	}
	set_pin(Xsens_enable_pin, 0);		// set Xsense to reset
	XbusParser_destroy(MTi->XBParser);
	free(MTi->ReceivedMessageStorage);
	MTi->Xstate = Xsens_Reset;
	return Xsens_OK;
}

Xsens_Status MTi_Update(MTi_data* MTi){
	Xsens_Status ret = Xsens_OK;

	if(MTi->HAL_UART_ErrorCallback_flag != 0){
		MTi->HAL_UART_ErrorCallback_flag = 0;
		ret =  Xsens_Failed_Receive;
	}
	if(MTi->RxCpltCallback_flag != 0){
		MTi->RxCpltCallback_flag = 0;
		if(MTi->cplt_mess_stored_flag){
			MTi->cplt_mess_stored_flag = 0;
			// set UART up to receive a new message
			MTi->RX_state = Xsens_receive_5;
			HAL_UART_Receive_IT(&huartMTi, (uint8_t *)MTi->aRxBuffer, 5);

			switch(MTi->ReceivedMessageStorage->mid){
			case XMID_Error:
				ErrorHandler(MTi->ReceivedMessageStorage);
				MTi->MT_Data_succerr[1]++;	//failed
				ret = Xsens_Failed_Update;
				break;
			case XMID_MTData2:
				MTi->MT_Data_succerr[0]++;	// success
				Decode_Packet(MTi);
				break;
			case XMID_ReqOutputConfigurationAck:
				PrintOutputConfig(MTi->ReceivedMessageStorage);
				break;
			default:
				// assume th other types are acks
				processAck(MTi,MTi->ReceivedMessageStorage->mid);
				break;
			}

			//TODO: check if this needs to be called
			TheAlligator(MTi->XBParser);
		}else{	// No full message received
			if(MTi->RX_state == Xsens_receive_5){
				if(MTi->XBParser->currentMessage.length){
					MTi->RX_state = Xsens_receive_rest;
					HAL_UART_Receive_IT(&huartMTi, (uint8_t *)MTi->aRxBuffer, MTi->XBParser->currentMessage.length);
				}
			}
		}
	}
	// set In-run Compas Calibration if not set before
	if(MTi->statusword & 0b00011000){
			//uprintf("calibrating Xsens.\n\r");
		}else if(MTi->started_icc == false){
			if(MTi_UseIcc(MTi) == Xsens_OK) {
				MTi->started_icc = true;
				uprintf("MTi: ICC started.\n\r");
			}
			else {
				uprintf("MTi: ICC failed.\n\r");
				return Xsens_Failed_Update;
			}
		}
	return ret;
}

Xsens_Status MTi_UART_RxCpltCallback(MTi_data* MTi){
	switch(MTi->RX_state){
	case Xsens_receive_5:
		XbusParser_parseBuffer(MTi->XBParser, MTi->aRxBuffer, 5);
		break;
	case Xsens_receive_rest:
		XbusParser_parseBuffer(MTi->XBParser, MTi->aRxBuffer, MTi->XBParser->currentMessage.length);
		break;
	default:
		return Xsens_Failed_Receive;
		break;
	}
	MTi->RxCpltCallback_flag = 1;
	return Xsens_OK;
}

void MTi_UART_ErrorCallback(MTi_data* MTi){
	MTi->HAL_UART_ErrorCallback_flag = 1;
}
///////////////////////////////////////////////////// PRIVATE FUNCTION IMPLEMENTATIONS

static Xsens_Status MTi_BuildConfig(MTi_data* MTi, enum XsDataIdentifier XDI, uint16_t frequency, bool complete){
	static uint8_t n_configs = 0;
	static struct OutputConfiguration config[MAX_XDI_CONFIGS];
	config[n_configs].dtype = XDI;
	config[n_configs++].freq =  frequency;
	if(complete){
		struct XbusMessage mess;
		mess.mid = XMID_SetOutputConfiguration;
		mess.length = n_configs;
		mess.data = &config;
		//uint16_t* mdptr = mess.data;
		//uprintf( "[%x %x] [%x %x] [%x %x]\n\r", *mdptr++, *mdptr++, *mdptr++, *mdptr++, *mdptr++, *mdptr++);
		SendXbusMessage(MTi,mess);
		n_configs = 0;
	}
	return Xsens_OK;
}

// copy information from uart buffer to storage so it can be decoded.
static void XBP_handleMessage(struct XbusMessage const* message){
	MTi.cplt_mess_stored_flag = 1;
	memcpy(MTi.ReceivedMessageStorage, message, MAX_RAW_MESSAGE_SIZE);
}

static void* XBP_allocateBuffer(size_t bufSize){
	return bufSize < MAX_RAW_MESSAGE_SIZE? malloc(bufSize) : NULL;
}

static void XBP_deallocateBuffer(void const* buffer){
	free((uint8_t(*)[MAX_RAW_MESSAGE_SIZE])buffer);
}

static Xsens_Status Decode_Packet(MTi_data* MTi){
	struct XbusMessage const* message = MTi->ReceivedMessageStorage;
	if (!message) return Xsens_Failed_Decode;
	

	// loop through all requested data/measurements
	// NOTE: not every value has to be in every packet
	for(int i = 0; i < MTi->configuration_total-1; i++){
		MTi_data_tuple conf = MTi->data_configurations[i];
		XbusMessage_getDataItem(conf.data, conf.ID, message);
	}
	return Xsens_OK;
}

static void SendXbusMessage(MTi_data* MTi, struct XbusMessage XbusMessage){
	size_t XbusMes_size =  XbusMessage_format(MTi->raw, (struct XbusMessage const*)&XbusMessage, XLLF_Uart);
	HAL_UART_Transmit/*_IT*/(&huartMTi, MTi->raw, XbusMes_size, 100);
}

static Xsens_Status MTi_GoToConfig(MTi_data* MTi){
	struct XbusMessage mess = {XMID_GoToConfig};
	SendXbusMessage(MTi,mess);
	MTi->expect_ack = XMID_GoToConfigAck;
	return Xsens_OK;

}

static Xsens_Status MTi_GoToMeasure(MTi_data* MTi){
	struct XbusMessage mess = {XMID_GoToMeasurement};
	SendXbusMessage(MTi,mess);
	MTi->expect_ack = XMID_GoToMeasurementAck;
	return Xsens_OK;
}

static Xsens_Status MTi_UseIcc(MTi_data* MTi){
	struct XbusMessage mess = {XMID_IccCommand, 1, XsIcc_Start};
	SendXbusMessage(MTi,mess);
	return Xsens_OK;
}

static Xsens_Status MTi_SetFilterProfile(MTi_data* MTi, enum XsFilterProfile filter){
	uint8_t data[2];
	XbusUtility_writeU16(data,  filter);
	struct XbusMessage mess = {XMID_SetFilterProfile, 2, data};
	SendXbusMessage(MTi,mess);
	MTi->expect_ack = XMID_SetFilterProfileAck;
	return Xsens_OK;
}

static Xsens_Status MTi_NoRotation(MTi_data* MTi, uint16_t seconds){
	uint8_t data[2];
	XbusUtility_writeU16(data, seconds);
	struct XbusMessage mess = {XMID_SetNoRotation, 2 , (void*)data};
	SendXbusMessage(MTi, mess);
	MTi->expect_ack = XMID_SetNoRotationAck;
	return Xsens_OK;
}

static Xsens_Status processAck(MTi_data* MTi, enum XsMessageId XMID){
	static uint16_t counter = 0;
	Xsens_Status ret = Xsens_OK;
	if (MTi->expect_ack != XMID){
		uprintf("MTi:received different ack, expected: %u, received: %u", MTi->expect_ack, XMID);
	}
	// process ack
	switch (XMID){
	case XMID_SetNoRotationAck: 	
		if (MT_DEBUG) uprintf("SetNoRotationAck.\n\r");
		MTi->expect_ack = 0;
		break;
	case XMID_ReqFilterProfileAck:	
		if (MT_DEBUG) uprintf("Req/Set FilterProfileAck.\n\r"); // request and set share the same value
		MTi->expect_ack = 0;
		break;
	case XMID_IccCommandAck:		
		if (MT_DEBUG) uprintf("IccCommandAck.\n\r");
		MTi->expect_ack = 0;
		break;
	case XMID_GoToConfigAck:		
		if (MT_DEBUG) uprintf("In config state.\n\r");
		MTi->Xstate = Xsens_Config;
		MTi->expect_ack = 0;
		break;
	case XMID_GoToMeasurementAck:
		if(MT_DEBUG) uprintf("In measurement state.\n\r");
		MTi->Xstate = Xsens_Measure;
		MTi->expect_ack = 0;
		break;
	case XMID_WakeUp:
		if(MT_DEBUG) uprintf("MTi: woke up.\n\r");
		// send Ack back to MTi to confirm
		struct XbusMessage XbusMes = { XMID_WakeUpAck};
		SendXbusMessage(MTi, XbusMes);
		// go to config state
		MTi->Xstate = Xsens_Config;
		return Xsens_OK;

	default:
		uprintf("MTi: received an uknown message of type: %u\n\r", XMID);
		break;
	}

	// if not 0 missed the ack
	if (MTi->expect_ack != 0){
		if (counter == 20){
			ret = Xsens_Failed_Config;
		}
		else{
			counter++;
			ret = Xsens_Wait_For_Ack;
		}
	}
	else{
		counter = 0;
		ret = Xsens_OK;
	}
	return ret;
}

static inline void ErrorHandler(struct XbusMessage const* message){
	if (!message)
		return;
	uprintf("ERROR: %02x\n\r", *(uint8_t *)(message->data));
}
