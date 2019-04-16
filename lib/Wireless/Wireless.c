/*
 * Wireless.c
 *
 *  Created on: 6 feb. 2019
 *      Author: Cas Doornkamp
 */

#include "Wireless.h"
#include "SX1280/SX1280_Constants.h"
#include "gpio_util.h"
#include "SX1280/SX1280.h"
#include <stdbool.h>
#include "msg_buff.h"
#include "..///TextOut///TextOut.h"


// make buffers
uint8_t TX_buffer[MAX_BUF_LENGTH] __attribute__((aligned(4)));
uint8_t RX_buffer[MAX_BUF_LENGTH] __attribute__((aligned(4)));

// init structs
SX1280_Settings set = {
        .frequency = 2400000000,
        .txPower = 31, //0x1F = 13dBm
		.packettype = PACKET_TYPE_FLRC,
        .TX_ramp_time = RADIO_RAMP_20_US,
		.periodBase = BASE_62_us,
        .periodBaseCount = 24,
		.syncWords = {0, 0, 0},
        .syncWordSensitivity = 1,
        .TXoffset = 128,
        .RXoffset = 0,
        .ModParam = {FLRC_BR_1_300_BW_1_2, FLRC_CR_1_0, BT_0_5},
        .PacketParam = {PREAMBLE_LENGTH_8_BITS, FLRC_SYNC_WORD_LEN_P32S, RX_MATCH_SYNC_WORD_1, PACKET_FIXED_LENGTH, RECEIVEPKTLEN, CRC_1_BYTE, NO_WHITENING},
        .DIOIRQ = {(TX_DONE|RX_DONE|RXTX_TIMEOUT), (TX_DONE|RX_DONE|RXTX_TIMEOUT), NONE, NONE}
    };
SX1280_Packet_Status PacketStat;

SX1280 * Wireless_Init(uint8_t channel, SPI_HandleTypeDef * WirelessSpi){
	SX1280 * SX = &SX1280_struct;// pointer to the global struct

    SX->SPI_used = false;

    SX->payloadLengh = 0;
	SX->RXbufferoffset = 0; // received location of the data buffer in SX1280
	SX->irqStatus = 0;      // last received IRQ status

    // set connections
    SX->SPI = WirelessSpi;
    SX->CS_pin = SPI3_CS;
    SX->busy_pin = SX_BUSY;
    set_pin(SX->CS_pin, HIGH);
    SX->IRQ_pin = SX_IRQ;

    // set buffer locations
    SX->RXbuf = RX_buffer;
    SX->TXbuf = TX_buffer;

    // link settings
    SX->SX_settings = &set;
    SX->SX_settings->channel = channel;
//    SX->SX_settings->syncWords[0] = robot_syncWord[RobotID];
    SX->Packet_status = &PacketStat;

    SX1280Setup(SX);

    setAutoFS(SX,true);

    return SX;
};


void SendAutoPacket(SX1280* SX, uint8_t * data, uint8_t Nbytes){
    writeBuffer(SX, FEEDBACK_HEADER, data, Nbytes);
    setAutoTX(SX, AUTO_TX_TIME);
};

void SendPacket(SX1280* SX, uint8_t * data, uint8_t Nbytes){
    clearIRQ(SX,ALL);
    writeBuffer(SX, FEEDBACK_HEADER, data, Nbytes);
    setTX(SX, SX->SX_settings->periodBase, SX->SX_settings->periodBaseCount);
}

void ReceivePacket(SX1280* SX){
    clearIRQ(SX,ALL);
    getRXBufferStatus(SX);
    readBuffer(SX, SX->payloadLengh);
    SX->expect_packet = true;
};

// -------------------------------------------- Handlers
void Wireless_IRQ_Handler(SX1280* SX, uint8_t * data, uint8_t Nbytes){
    uint16_t irq = getIRQ(SX);
    SX->irqStatus = irq;
    clearIRQ(SX,ALL);

    // process interrupts
    if(irq & TX_DONE){
    	//TextOut("I transmitted!!\n\r");
    	isTransmitting = false;
    	set_pin(LD_2, LOW);
    }

    if(irq & RX_DONE){
    	//TextOut("I received?\n\r");
    //	ReceivePacket(SX);
    }

    if(irq & SYNCWORD_VALID) {
    	//TextOut("SX_SYNC_VALID\n\r");
    }

    if(irq & SYNCWORD_ERROR) {
    	//TextOut("SX_SYNC_ERROR\n\r");
    }

    if(irq & CRC_ERROR) {
    	//TextOut("SX_CRC_ERROR\n\r");
    }

    if(irq & RXTX_TIMEOUT) {
    	isTransmitting = false;
    	//TextOut("SX_RXTX_TIMEOUT\n\r");
    }

    if(irq & PREAMBLE_DETECTED) {
    	//TextOut("SX_PREAMBLE\n\r");
    }
};

void Wireless_DMA_Handler(SX1280* SX, uint8_t* output){
    if (DMA_Callback(SX)) {
        if(SX->expect_packet){
            SX->expect_packet = false;
        }
    }
}
