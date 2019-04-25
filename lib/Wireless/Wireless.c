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
        .txPower = 31, // -18 + txPower = transmit power in dBm (13dBm max)
		.packettype = PACKET_TYPE_FLRC,
        .TX_ramp_time = RADIO_RAMP_20_US,
		.periodBase = BASE_62_us,
        .periodBaseCount = 24,
		.syncWords = {0x0, 0x0, 0x0},
		.syncWordTolerance = 2, // accepted wrong bits in a detected syncword
        .syncSensitivity = 1, // high sensitivity mode
        .TXoffset = 0x80,
        .RXoffset = 0x00,
        .ModParam = {FLRC_BR_1_300_BW_1_2, FLRC_CR_1_0, BT_0_5},
        .PacketParam = {PREAMBLE_LENGTH_16_BITS, FLRC_SYNC_WORD_LEN_P32S, RX_MATCH_SYNC_WORD_1, PACKET_FIXED_LENGTH, RECEIVEPKTLEN, CRC_1_BYTE, NO_WHITENING},
        .DIOIRQ = {(TX_DONE|RX_DONE|RXTX_TIMEOUT), (TX_DONE|RX_DONE|RXTX_TIMEOUT), NONE, NONE}
};
SX1280_Packet_Status PacketStat;

SX1280 * Wireless_Init(float channel, SPI_HandleTypeDef * WirelessSpi){
	SX1280 * SX = &SX1280_struct;// pointer to the global struct

    SX->SPI_used = false;

    SX->payloadLength = 0;
	SX->RXbufferoffset = 0; // received location of the data buffer in SX1280
	SX->irqStatus = 0;      // last received IRQ status

    // set connections
    SX->SPI = WirelessSpi;
    SX->CS_pin = SPI3_CS;
    set_pin(SX->CS_pin, HIGH);
    SX->BUSY_pin = SX_BUSY;
    SX->IRQ_pin = SX_IRQ;
    SX->RST_pin = SX_RST;

    // set buffer locations
    SX->RXbuf = RX_buffer;
    SX->TXbuf = TX_buffer;

    // link settings
    SX->SX_settings = &set;
    SX->SX_settings->channel = channel;
    SX->Packet_status = &PacketStat;

    SX1280Setup(SX); // SX1280 init procedure

    setAutoFS(SX,false); // to go or not to go FS after TX or RX

    return SX;
};


void SendAutoPacket(SX1280* SX, uint8_t * data, uint8_t Nbytes){
    writeBuffer(SX, data, Nbytes);
    setAutoTX(SX, AUTO_TX_TIME);
};

void SendPacket(SX1280* SX, uint8_t * data, uint8_t Nbytes){
	clearIRQ(SX,ALL);
    writeBuffer(SX, data, Nbytes);
    setTX(SX, SX->SX_settings->periodBase, SX->SX_settings->periodBaseCount);
}

void ReceivePacket(SX1280* SX){
	clearIRQ(SX,ALL);
    getRXBufferStatus(SX);
    readBuffer(SX, SX->payloadLength);
    SX->expect_packet = true;
};

// -------------------------------------------- Handlers
void Wireless_IRQ_Handler(SX1280* SX, uint8_t * data, uint8_t Nbytes){
    uint16_t irq = getIRQ(SX);
    SX->irqStatus = irq;
    getPacketStatus(SX);
//	char msg[20];
//	sprintf(msg, "rssi: %d\n\r", SX->Packet_status->RSSISync);
//	TextOut(msg);
//	sprintf(msg, "errors: %d\n\r", SX->Packet_status->errors);
//	TextOut(msg);
    clearIRQ(SX,ALL);

    // process interrupts
    if(irq & TX_DONE){
//    	TextOut("SX_IRQ TX_DONE\n\r");
    	isTransmitting = false;
    	toggle_pin(LD_3);
    }

    if(irq & RX_DONE){
//    	TextOut("SX_IRQ RX_DONE\n\r");
//    	toggle_pin(LD_2);
    	// if signal is strong, then receive packet; otherwise SendPacket() without changes
    	if (SX->Packet_status->RSSISync < 180) {
//    		ReceivePacket(SX);
    	}
    }

    if(irq & SYNCWORD_VALID) {
//    	TextOut("SX_IRQ SYNCWORD_VALID\n\r");
    }

    if(irq & SYNCWORD_ERROR) {
//    	TextOut("SX_IRQ SYNCWORD_ERROR\n\r");
    }

    if(irq & CRC_ERROR) {
//    	TextOut("SX_IRQ CRC_ERROR\n\r");
    }

    if(irq & RXTX_TIMEOUT) {
    	// did not receive packet from robot
    	isTransmitting = false;
//    	TextOut("SX_IRQ RXTX_TIMEOUT\n\r");
    }

    if(irq & PREAMBLE_DETECTED) {
//    	TextOut("SX_IRQ PREAMBLE_DETECTED\n\r");
    }
};

void Wireless_DMA_Handler(SX1280* SX, uint8_t* output){
	DMA_Callback(SX);
    if(SX->expect_packet){ // expecting incoming packet in the buffer
    	SX->expect_packet = false;
//    	TextOut("received packet: ");
//    	for (int i=0; i<13; i++) {
//    		TextOut(SX->RXbuf[3+i]);
//    		buf[i] = SX->RXbuf[3+i] + 1; // store received message+1 in buf[] and then send it back to BS
//    	}
//    	TextOut("\n\r");

        // SendPacket() with new buf[]
//		SX->SX_settings->syncWords[0] = robot_syncWord[sendToId];
//		setSyncWords(SX, SX->SX_settings->syncWords[0], 0x00, 0x00);
//        SendPacket(SX, buf, 13);
    }
}
