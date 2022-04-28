#include "SX1280.h"
#include "gpio_util.h"

////////////////////////////////////// Macros
// Lock the interface when assigning values to the interface and unlock when done reading. After the unlock the interface is available also for other function calls
#define LOCK(inter) while(inter->active_transfer){}; inter->active_transfer = true;
#define UNLOCK(inter) inter->active_transfer = false;


void SX1280WakeUp(SX1280_Interface* interface) {
	// reset the device
	set_pin(interface->Reset, LOW);
	HAL_Delay(1);
	set_pin(interface->Reset, HIGH);
    // Wait for interface to finish setting up
	while(read_pin(interface->BusyPin)) {}

    // claim active transfer as the previous active_transfer is not valid anymore after a reset
    interface->active_transfer = true;

    // send command
    interface->TXbuf[0] = GET_STATUS;
    set_pin(interface->CS, LOW);
    HAL_SPI_TransmitReceive(interface->SPI, interface->TXbuf, interface->RXbuf, 1, 100);
    set_pin(interface->CS, HIGH);

    UNLOCK(interface)

    while(read_pin(interface->BusyPin)) {}
    // TODO: Check if needed? After reset the default is standby
    setStandby(interface, 0);
}

// -------------------------------------------- Device Setup
/* 11.3 GetStatus Command, page 72 */
SX1280_Status getStatus(SX1280_Interface* interface){
    // wait till SPI is available
    LOCK(interface)
    // send command
    interface->TXbuf[0] = GET_STATUS;
    set_pin(interface->CS, LOW);
    HAL_StatusTypeDef stat = HAL_SPI_TransmitReceive(interface->SPI, interface->TXbuf, interface->RXbuf, 1, 100);
    set_pin(interface->CS, HIGH);
    // cast first byte of RXBuf (status) to struct
    interface->SX1280_status = *(SX1280_Status*)&interface->RXbuf[0];

    UNLOCK(interface)
    return interface->SX1280_status;
}

// ---------- Section 11.4 - Register Access, page 74
bool writeRegister(SX1280_Interface* interface, uint16_t address, void* data, uint8_t Nbytes){
    uint8_t* ptr = interface->TXbuf;
    // wait till send complete
    LOCK(interface)
    // make command
    *ptr++ = SX_WRITE_REG;
    *ptr++ = address >> 8;
    *ptr++ = address & 0xFF;
    memcpy(ptr,data,Nbytes);
    SendData(interface,Nbytes+3);
    UNLOCK(interface)
    return true;
}

void readRegister(SX1280_Interface* interface, uint16_t address, uint8_t* data, uint8_t Nbytes){
    uint8_t* ptr = interface->TXbuf;
    // wait till send complete
    LOCK(interface)
    // make command
    *ptr++ = SX_READ_REG;
    *ptr++ = address >> 8;
    *ptr++ = address & 0xFF;
    memset(ptr,0,Nbytes+1);
    SendData(interface,Nbytes+4);
    // copy to output
    memcpy(data,interface->RXbuf+4, Nbytes);
    UNLOCK(interface)
}

void modifyRegister(SX1280_Interface* interface, uint16_t address, uint8_t mask, uint8_t set_value) {
	uint8_t data;
	readRegister(interface, address, &data, 1);
	data &= ~mask;
	data |= set_value;
	writeRegister(interface, address, &data, 1);
}

// ---------- Section 11.5 - Data Buffer Operations, page 75
void writeBuffer(SX1280_Interface* interface, uint8_t TXoffset, uint8_t * data, uint8_t Nbytes){
    uint8_t* ptr = interface->TXbuf;
    // wait till send complete
    LOCK(interface)
    // make command
    *ptr++ = WRITE_BUF;
    *ptr++ = TXoffset;
    memcpy(ptr, data, Nbytes);
    SendData(interface, 2 + Nbytes);
    UNLOCK(interface)
}

void writeBuffer_DMA(SX1280_Interface* interface, uint8_t TXoffset, uint8_t * data, uint8_t Nbytes){
    uint8_t* ptr = interface->TXbuf;
    // wait till send complete
    LOCK(interface)
    // make command
    *ptr++ = WRITE_BUF;
    *ptr++ = TXoffset;
    // TODO: Maybe split writebuf in two parts, command and data. That way the dest* can immediatly be read from instead of an additianal memcpy before
    memcpy(ptr, data, Nbytes);
    SendData_DMA(interface, 2 + Nbytes);
}

void readBuffer(SX1280_Interface* interface, uint8_t RXoffset, uint8_t* dest, uint8_t Nbytes){
    uint8_t* ptr = interface->TXbuf;
    // wait till send complete
    LOCK(interface)
    // make command
    *ptr++ = READ_BUF;
    *ptr++ = RXoffset;
    memset(ptr, 0, Nbytes + 1);
    SendData(interface, Nbytes + 3);
    memcpy(dest, interface->RXbuf + 3, Nbytes);
    UNLOCK(interface)
}

void readBuffer_DMA(SX1280_Interface* interface, uint8_t RXoffset, uint8_t* dest, uint8_t Nbytes){
    uint8_t* ptr = interface->TXbuf;
    // wait till send complete
    LOCK(interface)
    // make command
    *ptr++ = READ_BUF;
    *ptr++ = RXoffset;
    memset(ptr, 0, Nbytes + 1);
    SendData_DMA(interface, Nbytes + 3);
    // TODO: Maybe split readbuf in two parts, command and data. That way the dest* can immediatly be written into instead of an additianal memcpy afterwards
}

// ---------- Section 11.6 - Radio Operation Modes, page 77
void setSleep(SX1280_Interface* interface, uint8_t config){
    // wait till send complete
    LOCK(interface)
    // make command
    interface->TXbuf[0] = SET_SLEEP;
    interface->TXbuf[1] = config;
    SendData(interface,2);
    UNLOCK(interface)
}

void setStandby(SX1280_Interface* interface, uint8_t config){
    // wait till send complete
    LOCK(interface)
    // make command
    interface->TXbuf[0] = SET_STANDBY;
    interface->TXbuf[1] = config;
    SendData(interface,2);
    UNLOCK(interface)
}

bool setFS(SX1280_Interface* interface){
    // wait till send complete
    LOCK(interface)
    // make command
    interface->TXbuf[0] = SET_FS;
    SendData(interface,1);
    UNLOCK(interface)
    return true;
}

/* 11.6.4 SetTx, page 79 */
/* The command setTX sets the device in Transmit mode. Clear IRQ status before using this command */
bool setTX(SX1280_Interface* interface, uint8_t base, uint16_t count){
	clearIRQ(interface, IRQ_ALL);
	// wait till send complete
	LOCK(interface)
    // make command
    interface->TXbuf[0] = SET_TX;
    interface->TXbuf[1] = base;
    interface->TXbuf[2] = count >> 8;
    interface->TXbuf[3] = count & 0xFF;
    SendData(interface,4);
    UNLOCK(interface)
    return true;
}

/* 11.6.5 SetRx, page 80 */
/* The command setRX sets the device in Receiver mode. The IRQ status should be cleared prior to using this command */
bool setRX(SX1280_Interface* interface, uint8_t base, uint16_t count){
	clearIRQ(interface, IRQ_ALL);
    // wait till send complete
    LOCK(interface)
    // make command
    interface->TXbuf[0] = SET_RX;
    interface->TXbuf[1] = base;
    interface->TXbuf[2] = count >> 8;
    interface->TXbuf[3] = count & 0xFF;
    SendData(interface,4);
    UNLOCK(interface)
    return true;
}

void setAutoTX(SX1280_Interface* interface, uint16_t wait_time){
    // wait till send complete
    LOCK(interface)
    // make command
    interface->TXbuf[0] = SET_AUTO_TX;
    interface->TXbuf[1] = wait_time >> 8;
    interface->TXbuf[2] = wait_time & 0xFF;
    SendData(interface,3);
    UNLOCK(interface)
}

/* 11.6.12 SetAutoFs, page 85 */
/* This feature modifies the chip behavior so that the state following a Rx or Tx operation is FS and not STDBY */
/* This feature is to be used to reduce the switching time between consecutive Rx and/or Tx operations */
void setAutoFS(SX1280_Interface* interface, bool enable){
    // wait till send complete
    LOCK(interface)
    // make command
    interface->TXbuf[0] = SET_AUTO_FS;
    interface->TXbuf[1] = enable;
    SendData(interface,2);
    UNLOCK(interface)
}

// ---------- Section 11.7 - Radio Configuration, page 85

/* 11.7.1 SetPacketType, page 85 */
void setPacketType(SX1280_Interface* interface, SX1280_PacketType type){
    // wait till send complete
    LOCK(interface)
    // make command
    interface->TXbuf[0] = SET_PACKET;
    interface->TXbuf[1] = type;
    SendData(interface,2);
    UNLOCK(interface)
}

/* 11.7.2 GetPacketType, page 86 */
uint8_t getPacketType(SX1280_Interface* interface){
    // wait till send complete
    LOCK(interface)
    // make command
    interface->TXbuf[0] = GET_PACKET;
    interface->TXbuf[1] = 0;
    interface->TXbuf[2] = 0;
    SendData(interface,3);
    uint8_t type = interface->RXbuf[2];
    UNLOCK(interface)
    return type;
}

/* 11.7.3 SetRfFrequency, page 87 */
void setRFFrequency(SX1280_Interface* interface, float frequency){
    uint8_t* ptr = interface->TXbuf;
    uint32_t freq = 0;
    // wait till send complete
    LOCK(interface)
    // make command
    *ptr++ = SET_RF_F;
    // set frequency
    freq = (uint32_t)((double)frequency*PLL_HZ_TO_STEP);
    //frequency =  0xB89D89; // hardcoded value for 2.4ghz
    *ptr++ = (freq >> 16) & 0xFF;
    *ptr++ = (freq >> 8 ) & 0xFF;
    *ptr++ = (freq      ) & 0xFF;
    SendData(interface,4);
    UNLOCK(interface)
}

/* 11.7.4 SetTxParams, page 87 */
void setTXParam(SX1280_Interface* interface, uint8_t power, SX1280_RampTime rampTime){
    // wait till send complete
    LOCK(interface)
    // make command
    interface->TXbuf[0] = SET_TX_PARAM;
    interface->TXbuf[1] = power;
    interface->TXbuf[2] = rampTime;
    SendData(interface,3);
    UNLOCK(interface)
}

/* 11.7.6 SetBufferBaseAddress, page 89 */
void setBufferBase(SX1280_Interface* interface, uint8_t tx_address, uint8_t rx_address){
    // wait till send complete
    LOCK(interface)
    // make command
    interface->TXbuf[0] = SET_BUF_RX_TX;
    interface->TXbuf[1] = tx_address;
    interface->TXbuf[2] = rx_address;
    SendData(interface,3);
    UNLOCK(interface)
}

/* 11.7.7 SetModulationParams, page 89 */
void setModulationParam(SX1280_Interface* interface, SX1280_ModulationParam* param){
    uint8_t* ptr = interface->TXbuf;
    size_t message_size = sizeof(SX1280_ModulationParam);
    // wait till send complete
    LOCK(interface)
    // make command
    *ptr++ = SET_MOD_PARAM;
    memcpy(ptr, param->bytes, message_size);
    SendData(interface, message_size + 1);
    UNLOCK(interface)
}

/* 11.7.8 SetPacketParams, page 90 */
void setPacketParam(SX1280_Interface* interface, SX1280_PacketParam* param){
    uint8_t* ptr = interface->TXbuf;
    size_t message_size = sizeof(SX1280_PacketParam);
    // wait till send complete
    LOCK(interface)
    // make command
    *ptr++ = SET_PACKET_PARAM;
    memcpy(ptr, param->bytes, message_size);
    SendData(interface, message_size + 1);
    UNLOCK(interface)
}

// Helper Function
void setChannel(SX1280_Interface* interface, float channel) {
	float frequency = (channel + 2400)*1000000;
	setRFFrequency(interface, frequency);
}

// ---------- Section 11.8 - Communication Status Information, page 92
/* 11.8.1 GetRxBufferStatus, page 92 */
void getRXBufferStatus(SX1280_Interface* interface, SX1280_RX_Buffer_Status* bufferStatus){
    uint8_t* ptr = interface->TXbuf;
    // wait till send complete
    LOCK(interface)
    // make command
    *ptr++ = GET_RX_BUF_STATUS;
    memset(ptr,0,3);
    SendData(interface,4);
    bufferStatus->payloadLength = interface->RXbuf[2];
    bufferStatus->bufferOffset = interface->RXbuf[3];
    UNLOCK(interface)
}

/* 11.8.2 GetPacketStatus, page 93 */
void getPacketStatus(SX1280_Interface* interface, SX1280_Packet_Status* status){
	uint8_t* ptr = interface->TXbuf;
    // wait till send complete
    LOCK(interface)
    // make command
    *ptr++ = GET_PACKET_STATUS;
    memset(ptr,0,6);
    SendData(interface,7);
    // assign to struct
    memcpy(status->bytes, interface->RXbuf+2, 5);
    UNLOCK(interface)
}

// ---------- Section 11.9 IRQ Handling, page 95
/* 11.9.1 SetDioIrqParams, page 96 */
void setDIOIRQParams(SX1280_Interface* interface, uint16_t DIOIRQ[4]){
	uint16_t tmp[4] ={0};
	for(int i = 0; i<4; i++){
		// reverse IRQ mask cuz it stores them flipped in memory (Big Endian <-> Little Endian)
		tmp[i] = ((DIOIRQ[i] << 8) & 0xFF00) | ((DIOIRQ[i] >> 8) & 0x00FF);
	}
	uint8_t* ptr = interface->TXbuf;
    // wait till send complete
    LOCK(interface)
    // make command
    *ptr++ = SET_DIO_IRQ_PARAM;
    memcpy(ptr,&tmp,8);
    SendData(interface,9);
    UNLOCK(interface)
}

/* 11.9.2 GetIrqStatus, page 97 */
void getIRQ(SX1280_Interface* interface, uint16_t* irq){
    uint8_t* ptr = interface->TXbuf;
    // wait till send complete
    LOCK(interface)
    // make command
    *ptr++ = GET_IRQ_STATUS;
    memset(ptr,0,3);
    SendData(interface,4);
    *irq = (interface->RXbuf[2] << 8) | interface->RXbuf[3];
    UNLOCK(interface)
    return;
}

/* 11.9.3 ClearIrqStatus, page 97 */
void clearIRQ(SX1280_Interface* interface, uint16_t mask){
    // wait till send complete
    LOCK(interface)
    // make command
    interface->TXbuf[0] = CLR_IRQ_STATUS;
    interface->TXbuf[1] = mask >> 8;
    interface->TXbuf[2] = mask & 0xFF;
    SendData(interface,3);
    UNLOCK(interface)
}

// ---------- Register Settings
/* 14.7.1 SetRegulatorMode, page 143 */
void setRegulatorMode(SX1280_Interface* interface, uint8_t mode) {
    // wait till send complete
    LOCK(interface)
    // make command
    interface->TXbuf[0] = SET_REGULATOR_MODE;
    interface->TXbuf[1] = mode;
    SendData(interface,2);
    UNLOCK(interface)
}

/* 4.2.1 Low Power Mode and High Sensitivity Mode, page 30 */
void setSyncSensitivity (SX1280_Interface* interface, uint8_t syncSensitivity) {
	if (syncSensitivity == 1) modifyRegister(interface, SYNC_SENS, 0xC0, 0xC0);
	else modifyRegister(interface, SYNC_SENS, 0xC0, 0x0);
}

SX1280_Error setSyncWord(SX1280_Interface* interface, uint8_t index, uint32_t syncWord) {
    uint16_t address = SYNCWORD1;
    switch(index){
        case 1: address = SYNCWORD1; break;
        case 2: address = SYNCWORD2; break;
        case 3: address = SYNCWORD3; break;
        default: return SX1280_PARAM_ERROR;
    }

    uint32_t rev_syncWord = __REV(syncWord);
    writeRegister(interface, address, &rev_syncWord, 4);
    return SX1280_OK;
}

void setSyncWordTolerance(SX1280_Interface* interface, uint8_t syncWordTolerance) {
	modifyRegister(interface, SYNC_TOL, 0x0F, syncWordTolerance & 0x0F);
}

void setCrcSeed(SX1280_Interface* interface, uint16_t seed){
    // Flip byte order to match how SX expects the data (Big Endian <-> Little Endian)
	uint16_t tmp = ((seed << 8) & 0xFF00) | ((seed >> 8) & 0x00FF);
    writeRegister(interface, CRC_INIT_MSB, &tmp, 2);
}
void setCrcPoly(SX1280_Interface* interface, uint16_t poly){
    // Flip byte order to match how SX expects the data (Big Endian <-> Little Endian)
    uint16_t tmp = ((poly << 8) & 0xFF00) | ((poly >> 8) & 0x00FF);
	writeRegister(interface, CRC_POLY_MSB, &tmp, 2);
}

// ---------- Callback Functions
void DMA_Callback(SX1280_Interface* interface, uint8_t* dest, uint8_t Nbytes){
    set_pin(interface->CS, HIGH);
    interface->SX1280_status = *(SX1280_Status*)interface->RXbuf; // store latest sx status
    // In case of readbuffer_DMA call copy received data into dest buffer
    if(dest){
        memcpy(dest, interface->RXbuf + 3, Nbytes);
    }
    UNLOCK(interface)
}

////////////////////////////////////// private functions
// Helper functions to Send/Receive data over SPI

// NOTE: Assumes callee has control of active_transfer
SX1280_Error SendData(SX1280_Interface* interface, uint8_t Nbytes){
	while(read_pin(interface->BusyPin)) {}
    SX1280_Error ret = SX1280_OK;
    
    // wait till ready
    while(interface->SPI->State != HAL_SPI_STATE_READY){}
    // send/receive data
    set_pin(interface->CS, LOW);
    HAL_StatusTypeDef stat = HAL_SPI_TransmitReceive(interface->SPI, interface->TXbuf, interface->RXbuf, Nbytes, 100);
    set_pin(interface->CS, HIGH);
    if(stat == HAL_ERROR || stat == HAL_TIMEOUT){
        ret = SX1280_FAIL;
    }

    // wait for interface to process command
    while(read_pin(interface->BusyPin)) {}
    return ret;
}

// NOTE: Assumes callee has control of active_transfer
SX1280_Error SendData_DMA(SX1280_Interface* interface, uint8_t Nbytes){
	while(read_pin(interface->BusyPin)) {}
	HAL_StatusTypeDef ret;
	// wait till ready
    while(interface->SPI->State != HAL_SPI_STATE_READY){}
    // send/receive data
    set_pin(interface->CS, LOW);
    ret = HAL_SPI_TransmitReceive_DMA(interface->SPI, interface->TXbuf, interface->RXbuf, Nbytes);
    if(ret == HAL_ERROR){
        ret = SX1280_FAIL;
    }
    return ret == HAL_OK;
}
