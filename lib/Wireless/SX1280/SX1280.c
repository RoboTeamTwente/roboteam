
#include "SX1280.h"
#include "gpio_util.h"

void SX1280Setup(SX1280* SX){
	SX1280WakeUp(SX);

    setPacketType(SX, SX->SX_settings->packettype); // packet type is set first!

//    char msg[10];
//    //TextOut("getPacketType ");
//    //TextOut(itoa(getPacketType(SX), msg, 16)); //
//    //TextOut("\n\r");

    setChannel(SX, SX->SX_settings->channel); // calls setRFFrequency() with freq=(channel+2400)*1000000
    //setRFFrequency(SX, SX->SX_settings->frequency);

    setBufferBase(SX, 0x80, 0x00);

    setModulationParam(SX);
    setPacketParam(SX);

    setTXParam(SX, SX->SX_settings->txPower, SX->SX_settings->TX_ramp_time);

    setSyncSensitivity (SX, SX->SX_settings->syncWordSensitivity);
    setSyncWordTolerance(SX, 5);
    setSyncWords(SX, SX->SX_settings->syncWords[0], SX->SX_settings->syncWords[1], SX->SX_settings->syncWords[2]);

    setDIOIRQParams(SX);

    setStandby(SX, 1); // set standby xosc after everything

    //setTX(SX, 0, 0);
    setFS(SX);

    getStatus(SX);
}

void SX1280WakeUp(SX1280* SX) {
	set_pin(SX_RST, LOW);
	HAL_Delay(1);
	set_pin(SX_RST, HIGH);
	HAL_Delay(1);

    while(SX->SPI_used){}
    SX->SPI_used = true;
    // send command
    SX->TXbuf[0] = GET_STATUS;
    SX->TXbuf[1] = 0;
    set_pin(SX->CS_pin, LOW);
    HAL_SPI_TransmitReceive(SX->SPI, SX->TXbuf, SX->RXbuf, 2, 100);
    set_pin(SX->CS_pin, HIGH);
    SX->SPI_used = false;
    while(read_pin(SX->busy_pin)) {}
    //TextOut("SX_WOKE\n\r");
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

    char status[100];
	sprintf (status, "SX_GETSTATUS %X\n\r", SX->RXbuf[0]);
	//TextOut(status);

//    if ((SX->RXbuf[0] & 0x01) == 1) { // check bit 0
    if (read_pin(SX->busy_pin) == 1) {
    	//TextOut("busy pin is high\n\r");
    }
//    	set_pin(LD_2, HIGH);
//    	 //TextOut("SX_BUSY\n\r");
//    	//else //TextOut("SX_BUSY_BUT_IS_IT?\n\r");
//    }
    else {
    	switch (SX->RXbuf[0]>>2 & 0x7) { // check bits 4:2
			case 0x1: // command has been terminated correctly
				//TextOut("SX_CMD_GOOD\n\r");
				break;
			case 0x2: // packet has been successfully received and data can be retrieved
				//TextOut("SX_DATA_IN\n\r");
				break;
			case 0x3: // command timeout, host should resend command
				//TextOut("SX_CMD_TIMOUT\n\r");
				break;
			case 0x4: // command was wrong (opcode or # of params)
				//TextOut("SX_CMD_ERR\n\r");
				break;
			case 0x5: // command execution failure
				toggle_pin(LD_3);
				//TextOut("SX_CMD_FAIL\n\r");
				break;
			case 0x6: // packet transmission completed
				toggle_pin(LD_2);
				//TextOut("SX_CMD_SENT\n\r");
				break;
    	}
    }

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

void setFS(SX1280* SX){
    // wait till send complete
    while(SX->SPI_used){}
    SX->SPI_used = true;
    // make command
    SX->TXbuf[0] = SET_FS;
    SendData(SX,1);
}

void setTX(SX1280* SX, uint8_t base, uint16_t count){
	getIRQ(SX);
	clearIRQ(SX, ALL);
    // wait till send complete
    while(SX->SPI_used){}
    SX->SPI_used = true;
    // make command
    SX->TXbuf[0] = SET_TX;
    SX->TXbuf[1] = base;
    SX->TXbuf[2] = count >> 8;
    SX->TXbuf[3] = count & 0xFF;

    if (SendData(SX,4)) {}//TextOut("setTX SendData completed\n\r");
    else {
    	char msg[100];
    	sprintf (msg, "setTX rxbuf[0] is %X\n\r", (SX->RXbuf[0]));
        //TextOut(msg);
    }
    getIRQ(SX);
}

void setRX(SX1280* SX, uint8_t base, uint16_t count){
	getIRQ(SX);
	clearIRQ(SX, ALL);
    // wait till send complete
    while(SX->SPI_used){}
    SX->SPI_used = true;
    // make command
    SX->TXbuf[0] = SET_RX;
    SX->TXbuf[1] = base;
    SX->TXbuf[2] = count >> 8;
    SX->TXbuf[3] = count & 0xFF;
    if (SendData(SX,4)) {}//TextOut("setRX SendData completed\n\r");
    else {
    	char msg[100];
    	sprintf (msg, "setRX rxbuf[0] is %X\n\r", (SX->RXbuf[0]>>2 & 0x7));
        //TextOut(msg);
    }
    getIRQ(SX);
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
    *ptr++ = (frequency >> 16) & 0xFF;
    *ptr++ = (frequency >> 8 ) & 0xFF;
    *ptr++ = (frequency      ) & 0xFF;
    SendData(SX,4);
}

void setChannel(SX1280* SX, uint8_t channel) {
	SX->SX_settings->channel = channel;

	uint32_t frequency = channel;
	frequency = (frequency + 2400)*1000000;

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
    SX->payloadLengh = SX->RXbuf[2];
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
	uint8_t* ptr = SX->TXbuf;
    // wait till send complete
    while(SX->SPI_used){}
    SX->SPI_used = true;
    // make command
    *ptr++ = SET_DIO_IRQ_PARAM;
    memcpy(ptr,SX->SX_settings->DIOIRQ,8);
    if (SendData(SX,9)) {}//TextOut("DIOIRQ succ\n\r");
    else {}//TextOut("DIOIRQ fucc\n\r");
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
	char msg[10];
	//TextOut("getIRQ ");
	//TextOut(itoa(((SX->RXbuf[2] <<8) | SX->RXbuf[3]), msg, 10));
	//TextOut("\n\r");
    return SX->irqStatus = (SX->RXbuf[2] <<8) | SX->RXbuf[3];
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

void writeBuffer(SX1280* SX, uint32_t header, uint8_t * data, uint8_t Nbytes){
    uint8_t* ptr = SX->TXbuf;
    // wait till send complete

    while(SX->SPI_used){}

    SX->SPI_used = true;
    // make command
    *ptr++ = WRITE_BUF;
    *ptr++ = SX->SX_settings->TXoffset;
    memcpy(ptr,&header,4);  // put header in front
    memcpy(ptr+4, data, Nbytes);
    SendData_DMA(SX, 2 + 4 + Nbytes);
}

void readBuffer(SX1280* SX, uint8_t Nbytes){
    uint8_t* ptr = SX->TXbuf;
    // wait till send complete
    while(SX->SPI_used){}
    SX->SPI_used = true;
    // make command
    *ptr++ = READ_BUF;
    *ptr++ = SX->SX_settings->RXoffset;
    memset(ptr, 0, Nbytes + 1);
    SendData_DMA(SX, Nbytes + 3);
}

// -------------------------------------------- Send / Receive
bool SendData(SX1280* SX, uint8_t Nbytes){
	while(read_pin(SX->busy_pin)) {}
    HAL_StatusTypeDef ret;
    // wait till ready
    while(SX->SPI->State != HAL_SPI_STATE_READY){}
    // send/receive data
    set_pin(SX->CS_pin, LOW);
    ret = HAL_SPI_TransmitReceive(SX->SPI, SX->TXbuf, SX->RXbuf, Nbytes, 100);
    set_pin(SX->CS_pin, HIGH);
    SX->SPI_used = false;
    while(read_pin(SX->busy_pin)) {}
    SX->TXbuf[0] = GET_STATUS;
    SX->TXbuf[1] = 0;
    set_pin(SX->CS_pin, LOW);
    HAL_SPI_TransmitReceive(SX->SPI, SX->TXbuf, SX->RXbuf, 2, 100);
    set_pin(SX->CS_pin, HIGH);
    SX->SX1280_status = SX->RXbuf[0];
    return (ret == HAL_OK && (SX->RXbuf[0]>>2 & 0x7) == 0x01);
}

bool SendData_DMA(SX1280* SX, uint8_t Nbytes){
	while(read_pin(SX->busy_pin)) {}
	HAL_StatusTypeDef ret;
	// wait till ready
    while(SX->SPI->State != HAL_SPI_STATE_READY){}
    // send/receive data
    set_pin(SX->CS_pin, LOW);
    ret = HAL_SPI_TransmitReceive_DMA(SX->SPI, SX->TXbuf, SX->RXbuf, Nbytes);
    return ret == HAL_OK;
}

bool DMA_Callback(SX1280* SX){
    set_pin(SX->CS_pin, HIGH);
    SX->SPI_used = false;
	return (SX->RXbuf[0]>>2 & 0x7) == 0x01;
}

// -------------------------------------------- Synchronization
void setSyncSensitivity (SX1280* SX, uint8_t syncWordSensitivity) {
	if (syncWordSensitivity == 1) modifyRegister(SX, SYNC_SENS, 0x0, 0xC0);
	else modifyRegister(SX, SYNC_SENS, 0xC0, 0x0);
}

bool setSyncWords (SX1280* SX, uint32_t syncWord_1, uint32_t syncWord_2, uint32_t syncWord_3) {
	uint32_t rev_syncWord = __REV(syncWord_1);
	if (!writeRegister(SX, SYNCWORD1, &rev_syncWord, 4))
		return false;

	rev_syncWord = __REV(syncWord_2);
	if (!writeRegister(SX, SYNCWORD2, &rev_syncWord, 4))
			return false;

	rev_syncWord = __REV(syncWord_3);
	if (!writeRegister(SX, SYNCWORD3, &rev_syncWord, 4))
		return false;

	return true;
}

void setSyncWordTolerance(SX1280* SX, uint8_t syncWordTolerance) {
	modifyRegister(SX, SYNC_TOL, 0x0F, syncWordTolerance & 0x0F);
}

//bool setSyncWord_1 (SX1280* SX, uint32_t word_1) {
//	word_1 = __REV(word_1);
//
//	if (!writeRegister(SX, SYNCWORD1, 4, &word_1))
//		return false;
//
//	return true;
//}
