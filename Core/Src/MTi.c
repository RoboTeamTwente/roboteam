/*
 * MTi.c
 *
 *  Created on: 25 april 2019
 *      Author: Cas Doornkamp
 */

#include <MTi/xbusutility.h>
#include <MTi/xsdeviceid.h>
#include <MTi/xbusdef.h>
#include "MTi/MTi_protocol.h"
#include "MTi.h"
#include <string.h>
#include "gpio_util.h"

#define TIME_OUT 1000U

///////////////////////////////////////////////////// data struct for MTi
MTi_data MTi_struct;
MTi_data* MTi;

uint8_t RXbuf[MAX_RAW_MESSAGE_SIZE] __attribute__((aligned(16)));
uint8_t TXbuf[MAX_RAW_MESSAGE_SIZE] __attribute__((aligned(16)));

///////////////////////////////////////////////////// DATA MEASUREMENT CONFIGURATIONS
MTi_data_tuple data_configurations[]={
	{.ID = XDI_PacketCounter	, .frequency = 100	, .data = &MTi_struct.packetcounter	},
	{.ID = XDI_Acceleration		, .frequency = 100	, .data = MTi_struct.acc			},
	{.ID = XDI_StatusWord		, .frequency = 10	, .data = &MTi_struct.statusword	},
	{.ID = XDI_EulerAngles		, .frequency = 100	, .data = MTi_struct.angles			},
	{.ID = XDI_RateOfTurn		, .frequency = 100	, .data = MTi_struct.gyr			}
	};

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
static Xsens_Status MTi_EnableAHS(MTi_data* MTi);

// State change functions
static Xsens_Status MTi_GoToConfig(MTi_data* MTi);
static Xsens_Status MTi_GoToMeasure(MTi_data* MTi);

// Debug functions
static Xsens_Status PrintOutputConfig(struct XbusMessage* message);
static inline void MTi_printf(char* format, ...);

///////////////////////////////////////////////////// PUBLIC FUNCTION IMPLEMENTATIONS

MTi_data* MTi_Init(uint16_t calibrate_time, enum XsFilterProfile filter_type){
	// init state for UART
	MTi = &MTi_struct;
	MTi->SPI = MTi_SPI;
	MTi->CS_pin = MTi_NSS_pin;
	MTi->SPI_busy = false;
	MTi->init_phase = true;
	MTi->RX_state = Xsens_receive_5;
	MTi->RxDataFlag = 0;
	MTi->ReceivedMessageStorage = malloc(MAX_RAW_MESSAGE_SIZE);    // Reserve memory to store the longest possible message
	MTi->TxBuffer = TXbuf;
	MTi->RxBuffer = RXbuf;

    // Create a structure to contain the callback functions
    struct XbusParserCallback XBP_callback = {
		.handleMessage = XBP_handleMessage,
		.allocateBuffer = XBP_allocateBuffer,
		.deallocateBuffer = XBP_deallocateBuffer
	};
	MTi->XBParser = XbusParser_create(&XBP_callback);
    if(MTi->XBParser == NULL){
		MTi_printf("failed create Xparser");
		return NULL;
	}

    // start MTi operation of specific sensors and their update frequencies
    set_Pin(MTi_RST_pin, false);
    set_Pin(MTi->CS_pin, true);
    HAL_Delay(1);
	set_Pin(MTi_RST_pin, true);
	//HAL_Delay(1);

	if(WaitForAck(MTi,XMID_WakeUp)){
		MTi_printf("failed go to config");
		return NULL;
	}
	uint8_t txbuf[6];
	uint8_t rxbuf[6];

	// set settings
	HAL_Delay(1);
	txbuf[0] = SetProtocol;
	txbuf[HEADER_LENGTH] = Notification;//| Measurement;
	while(MTi->SPI_busy){}
	MTi->SPI_busy = true;
	set_Pin(MTi->CS_pin, false);
	HAL_SPI_TransmitReceive(MTi->SPI, txbuf, rxbuf, 5, 100);
	set_Pin(MTi->CS_pin, true);
	MTi->SPI_busy = false;

	// verify settings
	HAL_Delay(1);
	txbuf[0] = ReadProtocol;
	while(MTi->SPI_busy){}
	MTi->SPI_busy = true;
	set_Pin(MTi->CS_pin, false);
	HAL_SPI_TransmitReceive(MTi->SPI, txbuf, rxbuf, 6, 100);
	set_Pin(MTi->CS_pin, true);
	MTi->SPI_busy = false;
	MTi_printf("settings: %u", rxbuf[5]);

	HAL_Delay(1);
	MTi_GoToConfig(MTi);
	if(WaitForAck(MTi,XMID_GoToConfigAck)){
		MTi_printf("failed go to config");
		return NULL;
	}
	// set measurement configurations
	HAL_Delay(1);
	MTi->data_configurations = data_configurations;
	MTi->configuration_total = sizeof(data_configurations)/sizeof(MTi_data_tuple);

	for(int i = 0; i < MTi->configuration_total; i++){
		MTi_data_tuple conf = data_configurations[i];
		bool last = (i == (MTi->configuration_total-1));
		MTi_BuildConfig(MTi, conf.ID, conf.frequency, last);
	}

	if(WaitForAck(MTi,XMID_ReqOutputConfigurationAck)){
		MTi_printf("config ack failed");
	}

	// set filter profile defined in XsFilterProfile
	// 0 = general, 1 = high magnetic dep, 2 = dynamic
	// 3 = assumes stable magnetic field, 4 = high dynamic magnetic field (unreferenced heading)
	HAL_Delay(1);
	if(MTi_SetFilterProfile(MTi, filter_type)){
		MTi_printf("failed SetFilter");
		return NULL;
	}
	// set MTi to measure state
	HAL_Delay(1);
	if(MTi_GoToMeasure(MTi)){
		MTi_printf("failed go to measure");
		return NULL;
	}
	MTi->init_phase = false;
	HAL_Delay(100);
	// calibrate rotation sensor for calibrate_time seconds
	if(MTi_NoRotation(MTi,calibrate_time)){
		MTi_printf("failed set no rotation");
 		return NULL;
	}

	HAL_Delay(1);
	if(MTi_UseIcc(MTi) == Xsens_OK) {
		MTi->started_icc = true;
		MTi_printf("ICC started");
	}
	else {
		MTi_printf("ICC failed");
		return NULL;
	}

	HAL_Delay(1);
	if(MTi_UseIcc(MTi) == Xsens_OK) {
		MTi->started_icc = true;
		MTi_printf("ICC started");
	}
	else {
		MTi_printf("ICC failed");
		return NULL;
	}

	// set settings
	HAL_Delay(1);
	txbuf[0] = SetProtocol;
	txbuf[HEADER_LENGTH] = Notification | Measurement;
	while(MTi->SPI_busy){}
	set_Pin(MTi->CS_pin, false);
	HAL_SPI_TransmitReceive(MTi->SPI, txbuf, rxbuf, 5, 100);
	set_Pin(MTi->CS_pin, true);

	MTi->acc[0] = 0.0f;
	MTi->acc[1] = 0.0f;
	MTi->acc[2] = 0.0f;
	return MTi;
}

Xsens_Status MTi_DeInit(MTi_data* MTi){
	MTi->RX_state = unitialized;
	if(HAL_SPI_Abort(MTi->SPI) != HAL_OK){
		MTi->Xstate = Xsens_Unknown;
		return Xsens_Failed_DeInit;
	}
	set_Pin(MTi_RST_pin, 0);		// set Xsense to reset
	XbusParser_destroy(MTi->XBParser);
	free(MTi->ReceivedMessageStorage);
	MTi->Xstate = Xsens_Reset;
	return Xsens_OK;
}

Xsens_Status MTi_SPI_RxCpltCallback(MTi_data* MTi){
	// deselect MTi
	set_Pin(MTi->CS_pin, true);
	MTi->RxBuffer[2] = XBUS_PREAMBLE;
	MTi->RxBuffer[3] = XBUS_MASTERDEVICE;
	XbusParser_parseBuffer(MTi->XBParser, MTi->RxBuffer+2, MTi->RxBuffer[5] +5);
	MTi->SPI_busy = false;
	Decode_Packet(MTi);
	TheAlligator(MTi->XBParser);
	return Xsens_OK;
}

void MTi_UART_ErrorCallback(MTi_data* MTi){
	MTi->HAL_UART_ErrorCallback_flag = 1;
}

Xsens_Status MTi_IRQ_Handler(MTi_data* MTi){
	Xsens_Status ret = Xsens_OK;

	while(MTi->SPI_busy){}
	MTi->SPI_busy = true;
	set_Pin(MTi->CS_pin, false);
	uint8_t txbuf[4+4] = {0x04,0x0};
	uint8_t rxbuf[4+4] = {0x0};
	HAL_SPI_TransmitReceive(MTi->SPI, txbuf, rxbuf, 8, 100);
	set_Pin(MTi->CS_pin, true);
	MTi->SPI_busy = false;

	//XbusParser_parseBuffer(MTi->XBParser, MTi->aRxBuffer, 5);
	PipeStat* pipe_status = &rxbuf[HEADER_LENGTH];
	if(pipe_status->notification_size){
		while(MTi->SPI_busy){}
		MTi->SPI_busy = true;
		uint8_t* ptr = MTi->TxBuffer;
		*ptr++ = ReadNotification;
		memset(ptr,0,2+pipe_status->notification_size);
		HAL_Delay(1);		// MTi cannot process commands this fast, so add a delay
		set_Pin(MTi->CS_pin, false);
		HAL_SPI_TransmitReceive(MTi->SPI, MTi->TxBuffer, MTi->RxBuffer, 4 + pipe_status->notification_size, 100);
		set_Pin(MTi->CS_pin, true);

		MTi->RxBuffer[2] = XBUS_PREAMBLE;
		MTi->RxBuffer[3] = XBUS_MASTERDEVICE;
		XbusParser_parseBuffer(MTi->XBParser, MTi->RxBuffer+2, 2 + pipe_status->notification_size);
		MTi->SPI_busy = false;

		if (MTi->ReceivedMessageStorage->mid == XMID_Error) {
			ErrorHandler(MTi->ReceivedMessageStorage);
			MTi->MT_Data_succerr[1]++;	//failed
			ret = Xsens_Failed_Update;
		}else{
			processAck(MTi);
			MTi->MT_Data_succerr[0]++;	// success
		}
		//TheAlligator(MTi->XBParser);
	}
	if(pipe_status->measurement_size && !MTi->init_phase){


		HAL_Delay(1);
		while(MTi->SPI_busy){}
		MTi->SPI_busy = true;
		uint8_t* ptr = MTi->TxBuffer;
		*ptr++ = ReadMeasurement;
		memset(ptr,0,2+pipe_status->measurement_size);

		set_Pin(MTi->CS_pin, false);
		if(HAL_SPI_TransmitReceive_DMA(MTi->SPI, MTi->TxBuffer, MTi->RxBuffer, 4 + pipe_status->measurement_size) != HAL_OK){
			MTi->SPI_busy = false;
		}
	}
	return ret;
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
		SendXbusMessage(MTi,mess);
		n_configs = 0;
	}
	return Xsens_OK;
}

// copy information from uart buffer to storage so it can be decoded.
static void XBP_handleMessage(struct XbusMessage const* message){
	memcpy(MTi->ReceivedMessageStorage, message, sizeof(struct XbusMessage));
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
	for(int i = 0; i < MTi->configuration_total; i++){
		MTi_data_tuple conf = data_configurations[i];
		XbusMessage_getDataItem(conf.data, conf.ID, message);
	}
	return Xsens_OK;
}

static HAL_StatusTypeDef SendXbusMessage(MTi_data* MTi, struct XbusMessage XbusMessage){
	size_t XbusMes_size = XbusMessage_format(MTi->TxBuffer, (struct XbusMessage const*)&XbusMessage, XLLF_Spi);
	bool SPI_status = false;
	uint32_t cnt = 0;
	while(MTi->SPI_busy){}
	MTi->SPI_busy = true;
	do{
		set_Pin(MTi->CS_pin, false);
		SPI_status = HAL_SPI_Transmit/*_IT*/(MTi->SPI, MTi->TxBuffer, XbusMes_size, 100);
		set_Pin(MTi->CS_pin, true);
		cnt++;
	}while( SPI_status && cnt < 20 );
	MTi->SPI_busy = false;
	return SPI_status;
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
	uint8_t cmd = XsIcc_Start;
	struct XbusMessage mess = {XMID_IccCommand, 1, &cmd};
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

static Xsens_Status MTi_EnableAHS(MTi_data* MTi){
	uint8_t data[8];
	for (int i=0; i<8; i++) {
		data[i] = 0;
	}
	data[3] = 10;
	struct XbusMessage mess = {XMID_SetOptionFlags, 8, data};
	SendXbusMessage(MTi,mess);
	return WaitForAck(MTi,XMID_SetOptionFlagsAck);
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
	MTi->LastAck = XMID;
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
	Putty_printf("MTi: ERROR: %02x", *(uint8_t*)message->data);
}

static inline Xsens_Status WaitForAck(MTi_data* MTi, enum XsMessageId XMID){
	bool timedout = false;
	uint32_t cnt = 0;
	while(MTi->LastAck != XMID && !timedout){
//		HAL_Delay(10);
//		timedout = 200 < cnt++;
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
