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
#include "PuTTY.h"


// make buffers
uint8_t TX_buffer[MAX_BUF_LENGTH] __attribute__((aligned(4)));
uint8_t RX_buffer[MAX_BUF_LENGTH] __attribute__((aligned(4)));

// init structs
SX1280_Settings set = {
        .frequency = 2400000000,
        .txPower = 31, //0x1F = 13dBm, P(dBm)=(-18)+txPower
		.packettype = PACKET_TYPE_FLRC,
        .TX_ramp_time = RADIO_RAMP_20_US,
		.periodBase = BASE_62_us,
        .periodBaseCount = 24,
		.syncWords = {0x08421442, 0x04210C21, 0x0CC31CC3},
        .syncSensitivity = 1,
        .TXoffset = 128,
        .RXoffset = 0,
        .ModParam = {FLRC_BR_1_300_BW_1_2, FLRC_CR_1_0, BT_0_5},
        .PacketParam = {PREAMBLE_LENGTH_16_BITS, FLRC_SYNC_WORD_LEN_P32S, RX_MATCH_SYNC_WORD_1, PACKET_FIXED_LENGTH, RECEIVEPKTLEN, CRC_2_BYTE, NO_WHITENING},
        .DIOIRQ = {(TX_DONE|RX_DONE|RXTX_TIMEOUT), (TX_DONE|RX_DONE|RXTX_TIMEOUT), NONE, NONE}
    };
SX1280_Packet_Status PacketStat;

SX1280 * Wireless_Init(float channel, SPI_HandleTypeDef * WirelessSpi){
	SX1280 * SX = &SX1280_struct;// pointer to the global struct

    SX->SPI_used = false;

    SX->payloadLengh = 0;
	SX->RXbufferoffset = 0; // received location of the data buffer in SX1280
	SX->irqStatus = 0;      // last received IRQ status

    // set connections
    SX->SPI = WirelessSpi;
    SX->CS_pin = SX_NSS_pin;
    SX->busy_pin = SX_BUSY_pin;
    set_pin(SX->CS_pin, HIGH);
    SX->IRQ_pin = SX_IRQ_pin;
    SX->RST_pin = SX_RST_pin;

    // set buffer locations
    SX->RXbuf = RX_buffer;
    SX->TXbuf = TX_buffer;

    // link settings
    SX->SX_settings = &set;
    SX->SX_settings->channel = channel;
//    SX->SX_settings->syncWords[0] = robot_syncWord[RobotID];
    SX->Packet_status = &PacketStat;

    SX1280Setup(SX);
    setSyncWords(SX, SX->SX_settings->syncWords[0], SX->SX_settings->syncWords[1], SX->SX_settings->syncWords[2]);
    setAutoFS(SX,false);
    setFS(SX);

//    bool succes = setFS(SX);
//    set_pin(LED1_pin, !succes);	// notify if something has gone wrong with wireless
    Putty_printf("SX started\n\r");

    return SX;
};


void SendAutoPacket(SX1280* SX, uint8_t * data, uint8_t Nbytes){
    writeBuffer(SX, data, Nbytes);
    setAutoTX(SX, AUTO_TX_TIME);
};

void SendPacket(SX1280* SX,uint32_t header, uint8_t * data, uint8_t Nbytes){
    clearIRQ(SX,ALL);
    uint8_t buf[13] = {0};
    uint8_t* ptr = buf;
//    if(header){
//    	// add room for Sync word in case it is requested
////    	header = __REV(header);
//		memcpy(ptr,&header,4);
//		Nbytes = Nbytes+4;
//		ptr = ptr+4;
//    }
    memcpy(ptr,data,Nbytes);
    writeBuffer(SX, data, Nbytes);
	  Putty_printf ("sending message: ");
	  for (int i=0; i<13; i++){
		  Putty_printf("%X", SX->TXbuf[2+i]);
	  }
	  Putty_printf("\n\r");
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
    getPacketStatus(SX);
	char msg[20];
	sprintf(msg, "rssi: %d\n\r", SX->Packet_status->RSSISync);
	Putty_printf(msg);
	sprintf(msg, "errors: %d\n\r", SX->Packet_status->errors);
	Putty_printf(msg);
    clearIRQ(SX,ALL);

    if (SX->Packet_status->RSSISync > 180) return; // leave if rssi too low

    // process interrupts
    if(irq & TX_DONE){
    	Putty_printf("TX_DONE...\n\r");
    }

    if(irq & RX_DONE){
    	ReceivePacket(SX);
    	Putty_printf("receiving...\n\r");
    }

    if(irq & SYNCWORD_VALID) {
    }

    if(irq & SYNCWORD_ERROR) {
    }

    if(irq & CRC_ERROR) {
    }

    if(irq & RXTX_TIMEOUT) {
    }

    if(irq & PREAMBLE_DETECTED) {
    }
};

void Wireless_DMA_Handler(SX1280* SX, uint8_t* output){
	DMA_Callback(SX);
	if (SX->expect_packet) {
		toggle_pin(LED3_pin);
		SX->expect_packet = false;
		setRX(SX, SX->SX_settings->periodBase, 0x0);
	}
}
