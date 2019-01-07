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
#include "tim.h"

#define TIME_OUT 1000U
///////////////////////////////////////////////////// DATA MEASUREMENT CONFIGURATIONS
MTi_data_tuple data_configurations[]={
	{.ID = XDI_PacketCounter	, .frequency = 100	, .data = &MTi.packetcounter},
	{.ID = XDI_FreeAcceleration	, .frequency = 100	, .data = MTi.acc			},
	{.ID = XDI_StatusWord		, .frequency = 10	, .data = &MTi.statusword	},
	{.ID = XDI_EulerAngles		, .frequency = 100	, .data = MTi.angles		}};

///////////////////////////////////////////////////// PRIVATE FUNCTION DECLERATIONS
// Xbus functions
static void XBP_handleMessage(struct XbusMessage const* message);
static void* XBP_allocateBuffer(size_t bufSize);
static void XBP_deallocateBuffer(void const* buffer);

// Uart interface handlers
static Xsens_Status MTi_BuildConfig(MTi_data* MTi, enum XsDataIdentifier XDI, uint16_t frequency, bool complete);
static HAL_StatusTypeDef SendXbusMessage(MTi_data* MTi, struct XbusMessage XbusMessage);
static Xsens_Status processAck(MTi_data* MTi);
static Xsens_Status MTi_ReqConfig(MTi_data* MTi);
static inline void ErrorHandler(struct XbusMessage const* message);
static Xsens_Status Decode_Packet(MTi_data* MTi);
static inline Xsens_Status WaitForAck(MTi_data* MTi, enum XsMessageId XMID);

// Calibration functions
static Xsens_Status MTi_NoRotation(MTi_data* MTi, uint16_t seconds);
static Xsens_Status MTi_SetFilterProfile(MTi_data* MTi, uint8_t filter);
static Xsens_Status MTi_UseIcc(MTi_data* MTi);

// State change functions
static Xsens_Status MTi_GoToConfig(MTi_data* MTi);
static Xsens_Status MTi_GoToMeasure(MTi_data* MTi);

// Debug functions
static Xsens_Status PrintOutputConfig(struct XbusMessage* message);
static inline void MTi_printf(char* format, ...);

///////////////////////////////////////////////////// PUBLIC FUNCTION IMPLEMENTATIONS

Xsens_Status MTi_Init(MTi_data* MTi, uint16_t calibrate_time, enum XsFilterProfile filter_type){
	// init state for UART
	MTi->RX_state = Xsens_receive_5;
	MTi->cplt_mess_stored_flag = 0;
	MTi->ReceivedMessageStorage = malloc(MAX_RAW_MESSAGE_SIZE);    // Reserve memory to store the longest possible message
	HAL_TIM_Base_Start_IT(&htim7);
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
		MTi_printf("failed create Xparser");
		return Xsens_Failed_Init;
	}

    // start MTi operation of specific sensors and their update frequencies
	set_pin(Xsens_enable_pin, 1);
	// TODO: Make this while loop not blocking
	if(WaitForAck(MTi,XMID_WakeUp)){
		MTi_printf("failed wakeup");
		return Xsens_Failed_Init;
	}
	MTi_printf("started MTi Operation");
	// assure config state
	MTi_GoToConfig(MTi);
	if(WaitForAck(MTi,XMID_GoToConfigAck)){
		MTi_printf("failed go to config");
		return Xsens_Failed_Init;
	}
	// set measurement configurations
	MTi->data_configurations = data_configurations;
	MTi->configuration_total = sizeof(data_configurations)/sizeof(MTi_data_tuple);

	for(int i = 0; i < MTi->configuration_total; i++){
		MTi_data_tuple conf = data_configurations[i];
		bool last = i == (MTi->configuration_total-1);
		MTi_BuildConfig(MTi, conf.ID, conf.frequency, last);
	}
	if(WaitForAck(MTi,XMID_ReqOutputConfigurationAck)){
		MTi_printf("config ack failed");
	}
	// set filter profile defined in XsFilterProfile
	// 0 = general, 1 = high magnetic dep, 2 = dynamic
	// 3 = assumes stable magnetic field, 4 = high dynamic magnetic field (unreferenced heading)
	if(MTi_SetFilterProfile(MTi, filter_type)){
		MTi_printf("failed SetFilter");
		return Xsens_Failed_Init;
	}

	// set MTi to measure state
	if(MTi_GoToMeasure(MTi)){
		MTi_printf("failed go to measure");
		return Xsens_Failed_Init;
	}
	HAL_Delay(50);
	// calibrate rotation sensor for calibrate_time seconds
	if(MTi_NoRotation(MTi,calibrate_time)){
		MTi_printf("failed set no rotation");
 		return Xsens_Failed_Init;
	}
	//MT_UseIcc();
	if(MTi_UseIcc(MTi) == Xsens_OK) {
		MTi->started_icc = true;
		MTi_printf("ICC started");
	}
	else {
		MTi_printf("ICC failed");
		return Xsens_Failed_Update;
	}
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
	if(MTi->RxCpltCallback_flag){
		MTi->RxCpltCallback_flag = 0;
		if(MTi->cplt_mess_stored_flag){	// received a full packet
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
			default:
				// assume the other types are acks
				processAck(MTi);
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
		//Putty_printf( "[%x %x] [%x %x] [%x %x]\n\r", *mdptr++, *mdptr++, *mdptr++, *mdptr++, *mdptr++, *mdptr++);
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
		MTi_data_tuple conf = data_configurations[i];
		XbusMessage_getDataItem(conf.data, conf.ID, message);
	}
	return Xsens_OK;
}

static HAL_StatusTypeDef  SendXbusMessage(MTi_data* MTi, struct XbusMessage XbusMessage){
	size_t XbusMes_size =  XbusMessage_format(MTi->raw, (struct XbusMessage const*)&XbusMessage, XLLF_Uart);
	bool UART_status = false;
	uint32_t cnt = 0;
	do{
		UART_status = HAL_UART_Transmit/*_IT*/(&huartMTi, MTi->raw, XbusMes_size, 100);
		cnt++;
	}while( UART_status && cnt < 20 );
	return UART_status;
}

static Xsens_Status MTi_GoToConfig(MTi_data* MTi){
	struct XbusMessage mess = {XMID_GoToConfig};
	SendXbusMessage(MTi,mess);
	return WaitForAck(MTi,XMID_GoToConfigAck);
}

static Xsens_Status MTi_ReqConfig(MTi_data* MTi){
	struct XbusMessage mess = {XMID_ReqOutputConfiguration};
	SendXbusMessage(MTi,mess);
	return WaitForAck(MTi,XMID_ReqOutputConfigurationAck);
}

static Xsens_Status MTi_GoToMeasure(MTi_data* MTi){
	struct XbusMessage mess = {XMID_GoToMeasurement};
	SendXbusMessage(MTi,mess);
	return WaitForAck(MTi,XMID_GoToMeasurementAck);
}

static Xsens_Status MTi_UseIcc(MTi_data* MTi){
	struct XbusMessage mess = {XMID_IccCommand, 1, XsIcc_Start};
	SendXbusMessage(MTi,mess);
	return WaitForAck(MTi,XMID_IccCommandAck);
}

static Xsens_Status MTi_SetFilterProfile(MTi_data* MTi, enum XsFilterProfile filter){
	uint8_t data[2];
	XbusUtility_writeU16(data,  filter);
	struct XbusMessage mess = {XMID_SetFilterProfile, 2, data};
	SendXbusMessage(MTi,mess);
	return WaitForAck(MTi,XMID_SetFilterProfileAck);
}

static Xsens_Status MTi_NoRotation(MTi_data* MTi, uint16_t seconds){
	uint8_t data[2];
	XbusUtility_writeU16(data, seconds);
	struct XbusMessage mess = {XMID_SetNoRotation, 2 , (void*)data};
	SendXbusMessage(MTi, mess);
	return WaitForAck(MTi,XMID_SetNoRotationAck);
}

static Xsens_Status processAck(MTi_data* MTi){
	enum XsMessageId XMID = MTi->ReceivedMessageStorage->mid;
	// process ack
	switch (XMID){
	case XMID_SetNoRotationAck: 	
		if (MT_DEBUG) MTi_printf("SetNoRotationAck");
		break;
	case XMID_ReqFilterProfileAck:	
		if (MT_DEBUG) MTi_printf("Req/Set FilterProfileAck"); // request and set share the same value
		break;
	case XMID_IccCommandAck:		
		if (MT_DEBUG) MTi_printf("IccCommandAck");
		break;
	case XMID_GoToConfigAck:		
		if (MT_DEBUG) MTi_printf("In config state");
		MTi->Xstate = Xsens_Config;
		break;
	case XMID_GoToMeasurementAck:
		if(MT_DEBUG) MTi_printf("In measurement state");
		MTi->Xstate = Xsens_Measure;
		break;
	case XMID_WakeUp:
		if(MT_DEBUG) MTi_printf("woke up");
		// send Ack back to MTi to confirm
		struct XbusMessage XbusMes = { XMID_WakeUpAck};
		SendXbusMessage(MTi, XbusMes);
		// go to config state
		MTi->Xstate = Xsens_Config;
		break;
	case XMID_ReqOutputConfigurationAck:
		PrintOutputConfig(MTi->ReceivedMessageStorage);
		break;
	default:
		MTi_printf("received an uknown message of type: %u", XMID);
		break;
	}
	return Xsens_OK;
}

static Xsens_Status PrintOutputConfig(struct XbusMessage* message){
	if (!message) return Xsens_Failed_Debug;
	if(MT_DEBUG){
		Putty_printf("MTiPrintOutputConfig:\n\r");
		uint16_t data = 0;
		if((data = XbusMessage_getOutputFreq(XDI_Temperature, message)))
			Putty_printf("XDI_Temperature:%u\n\r", data);
		if((data = XbusMessage_getOutputFreq(XDI_UtcTime, message)))
			Putty_printf("XDI_UtcTime:%u\n\r", data);
		if((data = XbusMessage_getOutputFreq(XDI_PacketCounter, message)))
			Putty_printf("XDI_PacketCounter:%u\n\r", data);
		if((data = XbusMessage_getOutputFreq(XDI_SampleTimeFine, message)))
			Putty_printf("XDI_SampleTimeFine:%u\n\r", data);
		if((data = XbusMessage_getOutputFreq(XDI_SampleTimeCoarse, message)))
			Putty_printf("XDI_SampleTimeCoarse:%u\n\r", data);
		if((data = XbusMessage_getOutputFreq(XDI_Quaternion, message)))
			Putty_printf("XDI_Quaternion:%u\n\r", data);
		if((data = XbusMessage_getOutputFreq(XDI_RotationMatrix, message)))
			Putty_printf("XDI_RotationMatrix:%u\n\r", data);
		if((data = XbusMessage_getOutputFreq(XDI_DeltaV, message)))
			Putty_printf("XDI_DeltaV:%u\n\r", data);
		if((data = XbusMessage_getOutputFreq(XDI_Acceleration, message)))
			Putty_printf("XDI_Acceleration:%x\n\r", data);
		if((data = XbusMessage_getOutputFreq(XDI_FreeAcceleration, message)))
			Putty_printf("XDI_FreeAcceleration:%u\n\r", data);
		if((data = XbusMessage_getOutputFreq(XDI_AccelerationHR, message)))
			Putty_printf("XDI_AccelerationHR:%u\n\r", data);
		if((data = XbusMessage_getOutputFreq(XDI_RateOfTurn, message)))
			Putty_printf("XDI_RateOfTurn:%u\n\r", data);
		if((data = XbusMessage_getOutputFreq(XDI_DeltaQ, message)))
			Putty_printf("XDI_DeltaQ:%u\n\r", data);
		if((data = XbusMessage_getOutputFreq(XDI_RateOfTurnHR, message)))
			Putty_printf("XDI_RateOfTurnHR:%u\n\r", data);
		if((data = XbusMessage_getOutputFreq(XDI_MagneticField, message)))
			Putty_printf("XDI_MagneticField:%u\n\r", data);
		if((data = XbusMessage_getOutputFreq(XDI_StatusByte, message)))
			Putty_printf("XDI_StatusByte:%u\n\r", data);
		if((data = XbusMessage_getOutputFreq(XDI_StatusWord, message)))
			Putty_printf("XDI_StatusWord:%u\n\r", data);
	}
	return Xsens_OK;
}

static inline void ErrorHandler(struct XbusMessage const* message){
	if (!message)
		return;
	Putty_printf("MTi: ERROR: %02x", *(uint8_t *)(message->data));
}

static inline Xsens_Status WaitForAck(MTi_data* MTi, enum XsMessageId XMID){
	HAL_UART_Receive_IT(&huartMTi,(uint8_t *)MTi->aRxBuffer, 5);
	bool timedout = false;
	uint32_t cnt = 0;
	while(MTi->ReceivedMessageStorage->mid != XMID && !timedout){
		HAL_Delay(10);
		//timedout = 200 < cnt++;
	}
	return timedout ? Xsens_Failed_Receive : Xsens_OK;
}

static inline void MTi_printf(char* format, ...){
	if(MT_DEBUG){
		char mes[100];
		va_list aptr;
		va_start(aptr, format); // give starting point of additional arguments
	    vsprintf(mes, format, aptr); // Copies and turns into string
	    va_end(aptr); // close list
	    Putty_printf("MTi: %s\n\r", mes);
	}
}
