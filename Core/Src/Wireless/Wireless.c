/*
 * Wireless.c
 *
 *  Created on: 6 feb. 2019
 *      Author: Cas Doornkamp
 */

#include "basestation.h"
#include "Wireless.h"
#include "SX1280_Constants.h"
#include "gpio_util.h"
#include "SX1280.h"
#include <stdbool.h>
#include "msg_buff.h"
#include "TextOut.h"


// make buffers
uint8_t SXTX_TX_buffer[MAX_BUF_LENGTH] __attribute__((aligned(4)));
uint8_t SXTX_RX_buffer[MAX_BUF_LENGTH] __attribute__((aligned(4)));
uint8_t SXRX_TX_buffer[MAX_BUF_LENGTH] __attribute__((aligned(4)));
uint8_t SXRX_RX_buffer[MAX_BUF_LENGTH] __attribute__((aligned(4)));

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
        .crcSeed = {0xAC, 0xB6}, // seed value of 0xACB6 = 0b'1010110010110110
        .crcPoly = {0x10, 0x21}, // poly of P16(x) = x16 + x12 + x5 + 1
        .TXoffset = 0x80,
        .RXoffset = 0x00,
        .ModParam = {FLRC_BR_1_300_BW_1_2, FLRC_CR_3_4, BT_0_5},
        .PacketParam = {PREAMBLE_LENGTH_24_BITS, FLRC_SYNC_WORD_LEN_P32S, RX_MATCH_SYNC_WORD_1, PACKET_FIXED_LENGTH, RECEIVEPKTLEN, CRC_2_BYTE, NO_WHITENING},
        .DIOIRQ = {(TX_DONE|RX_DONE|CRC_ERROR|RXTX_TIMEOUT), (TX_DONE|RX_DONE|RXTX_TIMEOUT), NONE, NONE}
};
SX1280_Packet_Status PacketStat;

SX1280 * Wireless_Init(float channel, SPI_HandleTypeDef * WirelessSpi, uint8_t mode){
	SX1280 * SX = (mode==0) ? &SX1280_TX_struct : &SX1280_RX_struct;// pointer to the global struct

    SX->SPI_used = false;

    SX->payloadLength = 0;
	SX->RXbufferoffset = 0; // received location of the data buffer in SX1280
	SX->irqStatus = 0;      // last received IRQ status

    // set connections
	// TO DO: FIND A MORE ELEGANT WAY OF DOING PIN ASSIGNMENT FOR TX/RX
    SX->SPI = WirelessSpi;
    SX->CS_pin = (mode==0) ? SX_TX_CS : SX_RX_CS;
    set_pin(SX->CS_pin, HIGH);
    SX->BUSY_pin = (mode==0) ? SX_TX_BUSY : SX_RX_BUSY;
    SX->IRQ_pin = (mode==0) ? SX_TX_IRQ : SX_RX_IRQ;
    SX->RST_pin = (mode==0) ? SX_TX_RST : SX_RX_RST;

    // set buffer locations
    SX->RXbuf = (mode==0) ? SXTX_RX_buffer : SXRX_RX_buffer;
    SX->TXbuf = (mode==0) ? SXTX_TX_buffer : SXRX_TX_buffer;

    // link settings
    SX->SX_settings = &set;
    SX->SX_settings->channel = channel;
    SX->Packet_status = &PacketStat;

    SX1280Setup(SX); // SX1280 init procedure

    setAutoFS(SX, true); // to go or not to go FS after TX or RX

    return SX;
};


void SendAutoPacket(SX1280* SX, uint8_t * data, uint8_t Nbytes){
    writeBuffer(SX, data, Nbytes);
    setAutoTX(SX, AUTO_TX_TIME);
};

void SendPacket(SX1280* SX, uint8_t * data, uint8_t Nbytes){
	clearIRQ(SX,ALL);
    writeBuffer(SX, data, Nbytes);
    // HAL_Delay(0);    
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
    
    clearIRQ(SX,ALL);

    /* Error in packet detected. Ignore */
    if(irq & CRC_ERROR) {
    	return;
    }

    /* Packet sent to robot */
    if(irq & TX_DONE){
    	isTransmitting = false;
    	toggle_pin(LD_TX);
    }

    if(irq & RX_DONE){
        /* It is possible that random noise can trigger the syncword. 
        * Syncword is 32 bits. Noise comes in at 2.4GHz. Syncword resets when wrong bit is received.
        * Expected length of wrong syncword is 1*0.5 + 2*0.25 + 3*0.125 + ... = 2
        * 2^32 combinations / (2400000000 / 2) syncwords = correct syncword every 3.57 seconds purely from noise
    	*/
        // Correct syncword from noise have a very weak signal 
    	// Threshold is at -160/2 = -80 dBm
        if (SX->Packet_status->RSSISync < 160) {
    		ReceivePacket(SX);
    	}else{
    		// not necessary to force setRX() here when configured in Rx Continuous mode
    		//setRX(SX, SX->SX_settings->periodBase, WIRELESS_RX_COUNT);
    	}
    	toggle_pin(LD_RX);
    }

    if(irq & RXTX_TIMEOUT) {
    	// did not receive packet from robot
    	isTransmitting = false;
    	toggle_pin(LD_LED3);
//    	TextOut("SX_IRQ RXTX_TIMEOUT\n\r");
    }
    
    // SYNCWORD_VALID interrupt not enabled in Wireless_Init. Never triggered
    if(irq & SYNCWORD_VALID) { }
    // SYNCWORD_ERROR interrupt not enabled in Wireless_Init. Never triggered
    if(irq & SYNCWORD_ERROR) { }
    // PREAMBLE_DETECTED interrupt not enabled in Wireless_Init. Never triggered
    if(irq & PREAMBLE_DETECTED) { }
};

void Wireless_DMA_Handler(SX1280* SX){
	DMA_Callback(SX);
    if(SX->expect_packet){ // expecting incoming packet in the buffer
    	SX->expect_packet = false;
    	// reset RX if not in continuous RX mode!
    	uint8_t id = SX->RXbuf[3];
        msgBuff[id].feedback[0] = PACKET_TYPE_ROBOT_FEEDBACK;
        memcpy(msgBuff[id].feedback+1, SX->RXbuf+3, PACKET_SIZE_ROBOT_FEEDBACK-1);
        msgBuff[id].isNewFeedback = true;
    }
}
