#include "SX1280.h"
#include "gpio_util.h"

uint32_t robot_syncWord[] = {
0x00000400, 0x04210C21, 0x08421442, 0x0CC31CC3,
0x10842484, 0x14A52CA5, 0x18C634C6, 0x1CD73CD7,
0x21084508, 0x25294D29, 0x298A554A, 0x2D6D5D6D,
0x318C658C, 0x35AD6DAD, 0x39CE75CE, 0x3DEF7DEF,

0x82108610, 0x86318E31, 0x8A529652, 0x8ED39ED3,
0x9294A694, 0x96B5AEB5, 0x9AD6B6D6, 0x9EE7BEE7,
0xB318C718, 0xB739CF39, 0xBB9AD75A, 0xBF7DDF7D,
0xC39CE79C, 0xC7BDEFBD, 0xCBDEF7DE, 0xCFFFFFFF
};

void SX1280Setup(SX1280* SX){
	SX1280WakeUp(SX); // reset, initialize SPI, set to STDBY_RC in order to configure SX1280

    setPacketType(SX, SX->SX_settings->packettype); // packet type is set first!

    setChannel(SX, SX->SX_settings->channel); // calls setRFFrequency() with freq=(channel+2400)*1000000

    setBufferBase(SX, SX->SX_settings->TXoffset, SX->SX_settings->RXoffset); // set offsets to write TX and RX data in the SX1280 buffer

    setModulationParam(SX); // flrc = bitrate&BW, coding rate, modulation

    setPacketParam(SX); // flrc = preamble length, syncword length, syncword match, header, payload length, crc length, whitening=no

    setSyncSensitivity (SX, SX->SX_settings->syncSensitivity); // enable/disable highest gain step for receive

    setSyncWordTolerance(SX, SX->SX_settings->syncWordTolerance); // set how many bits can go wrong in the sync word

    setCrcSeed(SX, SX->SX_settings->crcSeed[0], SX->SX_settings->crcSeed[1]); // set crc seed value (msb then lsb)

    setCrcPoly(SX, SX->SX_settings->crcPoly[0], SX->SX_settings->crcPoly[1]); // set crc polynomial (msb then lsb)

    setTXParam(SX, SX->SX_settings->txPower, SX->SX_settings->TX_ramp_time); // set TX power and ramp up time

    setDIOIRQParams(SX); // IRQ masks are stored in reverse, so reverse again
}

void SX1280WakeUp(SX1280* SX) {
	// reset the device
	set_pin(SX->RST_pin, LOW);
	HAL_Delay(1);
	set_pin(SX->RST_pin, HIGH);
	HAL_Delay(1);
	// trigger SPI interface
    while(SX->SPI_used){}
    SX->SPI_used = true;
    // send command
    SX->TXbuf[0] = GET_STATUS;
    SX->TXbuf[1] = 0;
    set_pin(SX->CS_pin, LOW);
    HAL_SPI_TransmitReceive(SX->SPI, SX->TXbuf, SX->RXbuf, 2, 100);
    set_pin(SX->CS_pin, HIGH);
    SX->SPI_used = false;
    while(read_pin(SX->BUSY_pin)) {}
    setStandby(SX, 0);
}

// -------------------------------------------- Device Setup
uint8_t getStatus(SX1280* SX){
    // wait till send complete
    while(SX->SPI_used){}
    SX->SPI_used = true;
    // send command
    SX->TXbuf[0] = GET_STATUS;
    SX->TXbuf[1] = 0;
    SendData(SX,2);
    return SX->SX1280_status = SX->RXbuf[0];
}

void setSleep(SX1280* SX, uint8_t config){
    // wait till send complete
    while(SX->SPI_used){}
    SX->SPI_used = true;
    // make command
    SX->TXbuf[0] = SET_SLEEP;
    SX->TXbuf[1] = config;
    SendData(SX,2);
}

void setStandby(SX1280* SX, uint8_t config){
    // wait till send complete
    while(SX->SPI_used){}
    SX->SPI_used = true;
    // make command
    SX->TXbuf[0] = SET_STANDBY;
    SX->TXbuf[1] = config;
    SendData(SX,2);
}

bool setFS(SX1280* SX){
    // wait till send complete
    while(SX->SPI_used){}
    SX->SPI_used = true;
    // make command
    SX->TXbuf[0] = SET_FS;
    return SendData(SX,1);
}

bool setTX(SX1280* SX, uint8_t base, uint16_t count){
	clearIRQ(SX, ALL);
	// wait till send complete
	while(SX->SPI_used){}
    SX->SPI_used = true;
    // make command
    SX->TXbuf[0] = SET_TX;
    SX->TXbuf[1] = base;
    SX->TXbuf[2] = count >> 8;
    SX->TXbuf[3] = count & 0xFF;
    return SendData(SX,4);
}

bool setRX(SX1280* SX, uint8_t base, uint16_t count){
	clearIRQ(SX, ALL);
    // wait till send complete
    while(SX->SPI_used){}
    SX->SPI_used = true;
    // make command
    SX->TXbuf[0] = SET_RX;
    SX->TXbuf[1] = base;
    SX->TXbuf[2] = count >> 8;
    SX->TXbuf[3] = count & 0xFF;
    return SendData(SX,4);
}
void setRXDuty(SX1280* SX, uint8_t base, uint16_t rxcount, uint16_t sleepcount){
    // wait till send complete
    while(SX->SPI_used){}
    SX->SPI_used = true;
    // make command
    SX->TXbuf[0] = SET_RX_DUTY;
    SX->TXbuf[1] = base;
    SX->TXbuf[2] = rxcount >> 8;
    SX->TXbuf[3] = rxcount & 0xFF;
    SX->TXbuf[4] = sleepcount >> 8;
    SX->TXbuf[5] = sleepcount & 0xFF;
    SendData(SX,6);
}
void setAutoFS(SX1280* SX, bool enable){
    // wait till send complete
    while(SX->SPI_used){}
    SX->SPI_used = true;
    // make command
    SX->TXbuf[0] = SET_AUTO_FS;
    SX->TXbuf[1] = enable;
    SendData(SX,2);
}
void setAutoTX(SX1280* SX, uint16_t wait_time){
    // wait till send complete
    while(SX->SPI_used){}
    SX->SPI_used = true;
    // make command
    SX->TXbuf[0] = SET_AUTO_TX;
    SX->TXbuf[1] = wait_time >> 8;
    SX->TXbuf[2] = wait_time & 0xFF;
    SendData(SX,3);
}

// -------------------------------------------- Packet Type / Params
void setRFFrequency(SX1280* SX, uint32_t frequency){
    uint8_t* ptr = SX->TXbuf;
    // wait till send complete
    while(SX->SPI_used){}
    SX->SPI_used = true;
    // make command
    *ptr++ = SET_RF_F;
    // set frequency
    frequency = (uint32_t)((double)frequency*PLL_STEP);
    //frequency =  0xB89D89; // hardcoded value for 2.4ghz
    *ptr++ = (frequency >> 16) & 0xFF;
    *ptr++ = (frequency >> 8 ) & 0xFF;
    *ptr++ = (frequency      ) & 0xFF;
    SendData(SX,4);
}

void setChannel(SX1280* SX, float channel) {
	SX->SX_settings->channel = channel;
	uint32_t frequency = (channel + 2400)*1000000;
	setRFFrequency(SX, frequency);
}

void setPacketType(SX1280* SX, uint8_t type){
    // wait till send complete
    while(SX->SPI_used){}
    SX->SPI_used = true;
    // make command
    SX->TXbuf[0] = SET_PACKET;
    SX->TXbuf[1] = type;
    SendData(SX,2);
}

uint8_t getPacketType(SX1280* SX){
    // wait till send complete
    while(SX->SPI_used){}
    SX->SPI_used = true;
    // make command
    SX->TXbuf[0] = GET_PACKET;
    SX->TXbuf[1] = 0;
    SX->TXbuf[2] = 0;
    SendData(SX,3);
    return SX->RXbuf[2];
}

void setTXParam(SX1280* SX, uint8_t power, uint8_t rampTime){
    // wait till send complete
    while(SX->SPI_used){}
    SX->SPI_used = true;
    // make command
    SX->TXbuf[0] = SET_TX_PARAM;
    SX->TXbuf[1] = power;
    SX->TXbuf[2] = rampTime;
    SendData(SX,3);
}

void setRegulatorMode(SX1280* SX, uint8_t mode) {
    // wait till send complete
    while(SX->SPI_used){}
    SX->SPI_used = true;
    // make command
    SX->TXbuf[0] = SET_REGULATOR_MODE;
    SX->TXbuf[1] = mode;
    SendData(SX,2);
}

void setBufferBase(SX1280* SX, uint8_t tx_address, uint8_t rx_address){
    // wait till send complete
    while(SX->SPI_used){}
    SX->SPI_used = true;
    // make command
    SX->TXbuf[0] = SET_BUF_RX_TX;
    SX->TXbuf[1] = tx_address;
    SX->TXbuf[2] = rx_address;
    SendData(SX,3);
}

void setModulationParam(SX1280* SX){
    uint8_t* ptr = SX->TXbuf;
    // wait till send complete
    while(SX->SPI_used){}
    SX->SPI_used = true;
    // make command
    *ptr++ = SET_MOD_PARAM;
    memcpy(ptr,SX->SX_settings->ModParam,3);
    SendData(SX,4);
}

void setPacketParam(SX1280* SX){
    uint8_t* ptr = SX->TXbuf;
    // wait till send complete
    while(SX->SPI_used){}
    SX->SPI_used = true;
    // make command
    *ptr++ = SET_PACKET_PARAM;
    memcpy(ptr,SX->SX_settings->PacketParam,7);
    SendData(SX,8);
}

// -------------------------------------------- Status
void getRXBufferStatus(SX1280* SX){
    uint8_t* ptr = SX->TXbuf;
    // wait till send complete
    while(SX->SPI_used){}
    SX->SPI_used = true;
    // make command
    *ptr++ = GET_RX_BUF_STATUS;
    memset(ptr,0,3);
    SendData(SX,4);
    SX->payloadLength = SX->RXbuf[2];
    SX->RXbufferoffset = SX->RXbuf[3];
}

void getPacketStatus(SX1280* SX){
	uint8_t* ptr = SX->TXbuf;
    // wait till send complete
    while(SX->SPI_used){}
    SX->SPI_used = true;
    // make command
    *ptr++ = GET_PACKET_STATUS;
    memset(ptr,0,6);
    SendData(SX,7);
    // assign to struct
    memcpy(SX->Packet_status, SX->RXbuf+2, 5);
}

// -------------------------------------------- Interrupt
void setDIOIRQParams(SX1280* SX){
	uint16_t tmp[4] ={0};
	for(int i = 0; i<4; i++){
		// reverse IRQ mask cuz it stores them flipped in memory
		tmp[i] = __REV16(SX->SX_settings->DIOIRQ[i]);
	}
	uint8_t* ptr = SX->TXbuf;
    // wait till send complete
    while(SX->SPI_used){}
    SX->SPI_used = true;
    // make command
    *ptr++ = SET_DIO_IRQ_PARAM;
    memcpy(ptr,&tmp,8);
    SendData(SX,9);
}

uint16_t getIRQ(SX1280* SX){
    uint8_t* ptr = SX->TXbuf;
    // wait till send complete
    while(SX->SPI_used){}
    SX->SPI_used = true;
    // make command
    *ptr++ = GET_IRQ_STATUS;
    memset(ptr,0,3);
    SendData(SX,4);
    return SX->irqStatus = (SX->RXbuf[2] << 8) | SX->RXbuf[3];
}

void clearIRQ(SX1280* SX, uint16_t mask){
    // wait till send complete
    while(SX->SPI_used){}
    SX->SPI_used = true;
    // make command
    SX->TXbuf[0] = CLR_IRQ_STATUS;
    SX->TXbuf[1] = mask >> 8;
    SX->TXbuf[2] = mask & 0xFF;
    SendData(SX,3);
}

// -------------------------------------------- Transfer Data
bool writeRegister(SX1280* SX, uint16_t address, void* data, uint8_t Nbytes){
    uint8_t* ptr = SX->TXbuf;
    // wait till send complete
    while(SX->SPI_used){}
    SX->SPI_used = true;
    // make command
    *ptr++ = SX_WRITE_REG;
    *ptr++ = address >> 8;
    *ptr++ = address & 0xFF;
    memcpy(ptr,data,Nbytes);
    return SendData(SX,Nbytes+3);
}

void readRegister(SX1280* SX, uint16_t address, uint8_t* data, uint8_t Nbytes){
    uint8_t* ptr = SX->TXbuf;
    // wait till send complete
    while(SX->SPI_used){}
    SX->SPI_used = true;
    // make command
    *ptr++ = SX_READ_REG;
    *ptr++ = address >> 8;
    *ptr++ = address & 0xFF;
    memset(ptr,0,Nbytes+1);
    SendData(SX,Nbytes+4);
    // copy to output
    memcpy(data,SX->RXbuf+4, Nbytes);
}

void modifyRegister(SX1280* SX, uint16_t address, uint8_t mask, uint8_t set_value) {
	uint8_t data;
	readRegister(SX, address, &data, 1);
	data &= ~mask;
	data |= set_value;
	writeRegister(SX, address, &data, 1);
}

bool writeBuffer(SX1280* SX, uint8_t * data, uint8_t Nbytes){
    uint8_t* ptr = SX->TXbuf;
    // wait till send complete
    while(SX->SPI_used){}
    SX->SPI_used = true;
    // make command
    *ptr++ = WRITE_BUF;
    *ptr++ = SX->SX_settings->TXoffset;
    memcpy(ptr, data, Nbytes);
    return SendData_DMA(SX, 2 + Nbytes);
}

void readBuffer(SX1280* SX, uint8_t Nbytes){
    uint8_t* ptr = SX->TXbuf;
    // wait till send complete
    while(SX->SPI_used){}
    SX->SPI_used = true;
    // make command
    *ptr++ = READ_BUF;
    *ptr++ = SX->RXbufferoffset;
    memset(ptr, 0, Nbytes + 1);
    SendData_DMA(SX, Nbytes + 3);
}

// -------------------------------------------- Send / Receive
bool SendData(SX1280* SX, uint8_t Nbytes){
	while(read_pin(SX->BUSY_pin)) {}
    HAL_StatusTypeDef ret;
    // wait till ready
    while(SX->SPI->State != HAL_SPI_STATE_READY){}
    // send/receive data
    set_pin(SX->CS_pin, LOW);
    HAL_SPI_TransmitReceive(SX->SPI, SX->TXbuf, SX->RXbuf, Nbytes, 100);
    set_pin(SX->CS_pin, HIGH);
    // wait for SX to process command
    while(read_pin(SX->BUSY_pin)) {}
    // check status of the processed command
    while(SX->SPI->State != HAL_SPI_STATE_READY){}
    SX->TXbuf[0] = GET_STATUS;
    SX->TXbuf[1] = 0;
    set_pin(SX->CS_pin, LOW);
    ret = HAL_SPI_TransmitReceive(SX->SPI, SX->TXbuf, SX->RXbuf, 2, 100);
    set_pin(SX->CS_pin, HIGH);
    SX->SPI_used = false;
    SX->SX1280_status = SX->RXbuf[0];
    // return SPI status and SX status==command processed successfully
    uint8_t status = SX->RXbuf[0]>>2 & 0x7;
    bool ret_bool = false;
    ret_bool |= status == 0x3; // if status is 3||4||5
    ret_bool |= status == 0x4; // then something went wrong
    ret_bool |= status == 0x5;
    return (ret == HAL_OK && !ret_bool);
}

bool SendData_DMA(SX1280* SX, uint8_t Nbytes){
	while(read_pin(SX->BUSY_pin)) {}
	HAL_StatusTypeDef ret;
	// wait till ready
    while(SX->SPI->State != HAL_SPI_STATE_READY){}
    // send/receive data
    set_pin(SX->CS_pin, LOW);
    ret = HAL_SPI_TransmitReceive_DMA(SX->SPI, SX->TXbuf, SX->RXbuf, Nbytes);
    return ret == HAL_OK;
}

void DMA_Callback(SX1280* SX){
    set_pin(SX->CS_pin, HIGH);
    SX->SPI_used = false;
    SX->SX1280_status = SX->RXbuf[0]; // store sx status
}

// -------------------------------------------- Synchronization
void setSyncSensitivity (SX1280* SX, uint8_t syncSensitivity) {
	if (syncSensitivity == 1) modifyRegister(SX, SYNC_SENS, 0x0, 0xC0);
	else modifyRegister(SX, SYNC_SENS, 0xC0, 0x0);
}

bool setSyncWords (SX1280* SX, uint32_t syncWord_1, uint32_t syncWord_2, uint32_t syncWord_3) {
	uint32_t rev_syncWord;
	bool success = false;
	// pass 0x00 as syncword to not overwrite stored value
	if (syncWord_1 != 0) {
		rev_syncWord = __REV(syncWord_1);
		if (writeRegister(SX, SYNCWORD1, &rev_syncWord, 4))
			success = true;
	}
	if (syncWord_2 != 0) {
		rev_syncWord = __REV(syncWord_2);
		if (writeRegister(SX, SYNCWORD2, &rev_syncWord, 4))
			success = true;
	}
	if (syncWord_3 != 0) {
		rev_syncWord = __REV(syncWord_3);
		if (writeRegister(SX, SYNCWORD3, &rev_syncWord, 4))
			success = true;
	}
	return success;
}

void setSyncWordTolerance(SX1280* SX, uint8_t syncWordTolerance) {
	modifyRegister(SX, SYNC_TOL, 0x0F, syncWordTolerance & 0x0F);
}

//bool setSyncWord_1 (SX1280* SX, uint32_t word_1) {
//	word_1 = __REV(word_1);
//	if (!writeRegister(SX, SYNCWORD1, 4, &word_1))
//		return false;
//	return true;
//}
void setCrcSeed(SX1280* SX, uint8_t seed_msb, uint8_t seed_lsb){
	writeRegister(SX, CRC_INIT_MSB, &seed_msb, 1);
	writeRegister(SX, CRC_INIT_LSB, &seed_lsb, 1);
}
void setCrcPoly(SX1280* SX, uint8_t poly_msb, uint8_t poly_lsb){
	writeRegister(SX, CRC_POLY_MSB, &poly_msb, 1);
	writeRegister(SX, CRC_POLY_LSB, &poly_lsb, 1);
}
