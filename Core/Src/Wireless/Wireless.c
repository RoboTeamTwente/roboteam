/*
 * Wireless.c
 *
 *  Created on: 6 feb. 2019
 *      Author: Cas Doornkamp
 */

#include "Wireless.h"
#include "SX1280_Constants.h"
#include "gpio_util.h"
#include "SX1280.h"
#include <stdbool.h>
#include "PuTTY.h"

static bool isWirelessConnected = false; // boolean to check whether we have a wireless connection or not
static bool isWirelessTransmitting = false; // boolean to check whether we are transmitting feedback

// make buffers
uint8_t TX_buffer[MAX_BUF_LENGTH] __attribute__((aligned(16)));
uint8_t RX_buffer[MAX_BUF_LENGTH] __attribute__((aligned(16)));

SX1280 SX1280_struct;
SX1280* SX; // pointer to the datastruct

bool wirelessFeedback = true; // boolean to enable or disable wireless feedback

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
        // .PacketParam = {PREAMBLE_LENGTH_24_BITS, FLRC_SYNC_WORD_LEN_P32S, RX_MATCH_SYNC_WORD_1, PACKET_VARIABLE_LENGTH, PACKET_SIZE_ROBOT_FEEDBACK, CRC_2_BYTE, NO_WHITENING},
        .PacketParam = {PREAMBLE_LENGTH_24_BITS, FLRC_SYNC_WORD_LEN_P32S, RX_MATCH_SYNC_WORD_1, PACKET_VARIABLE_LENGTH, 100, CRC_2_BYTE, NO_WHITENING},
        .DIOIRQ = {(TX_DONE|RX_DONE|CRC_ERROR|RXTX_TIMEOUT), (TX_DONE|RX_DONE|CRC_ERROR|RXTX_TIMEOUT), NONE, NONE}
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
    SX->CS_pin = SX_NSS_pin;
    set_Pin(SX->CS_pin, HIGH);
    SX->BUSY_pin = SX_BUSY_pin;
    SX->IRQ_pin = SX_IRQ_pin;
    SX->RST_pin = SX_RST_pin;

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

void Wireless_DeInit() {
	// Not necessary, till we get the feedback
}

void SendPacket(SX1280* SX, uint8_t * data, uint8_t Nbytes){
	clearIRQ(SX,ALL);

    // If the packet that we're sending has a different size than the previous packet, we have to update the packet size in the SX1280
    // Table 14-38: Payload Length Definition in FLRC Packet, page 124
    if(SX->SX_settings->PacketParam[4] != Nbytes){
        SX->SX_settings->PacketParam[4] = Nbytes;
        setPacketParam(SX);
    }
    // Not sure if this is needed, but just to be sure
    clearIRQ(SX,ALL);

    writeBuffer(SX, data, Nbytes);
    setTX(SX, SX->SX_settings->periodBase, SX->SX_settings->periodBaseCount);
}

void ReceivePacket(SX1280* SX){
	clearIRQ(SX,ALL);
    getRXBufferStatus(SX);
    SX->expect_packet = true;
    readBuffer(SX, SX->payloadLength);
};

bool checkWirelessConnection() {
	return isWirelessConnected;
}

// -------------------------------------------- Handlers
void Wireless_IRQ_Handler(SX1280* SX, uint8_t * data, uint8_t Nbytes){
    uint16_t irq = getIRQ(SX);
    SX->irqStatus = irq;
    getPacketStatus(SX);
    clearIRQ(SX,ALL);

    if(irq & CRC_ERROR) {
    	setRX(SX, SX->SX_settings->periodBase, WIRELESS_RX_COUNT);
    	return;
    }

    // process interrupts
    if(irq & TX_DONE){
    	isWirelessTransmitting = false;
    	//toggle_Pin(LED5_pin);
    	// start listening for a packet again
    	setChannel(SX, WIRELESS_COMMAND_CHANNEL); // set to channel 40 for basestation to robot
		SX->SX_settings->syncWords[0] = robot_syncWord[get_Id()];
		setSyncWords(SX, SX->SX_settings->syncWords[0], 0x00, 0x00);

        // If the packet that we're sending has a different size than the previous packet, we have to update the packet size in the SX1280
        // Table 14-38: Payload Length Definition in FLRC Packet, page 124
        SX->SX_settings->PacketParam[4] = 127; // TODO Create a #define for this, so that it works with RXoffset and TXoffset
        setPacketParam(SX);

		setRX(SX, SX->SX_settings->periodBase, WIRELESS_RX_COUNT);
    }

    if(irq & RX_DONE){
    	isWirelessConnected = true;
    	toggle_Pin(LED6_pin);
    	// if signal is strong, then receive packet; otherwise wait for packets
    	if (SX->Packet_status->RSSISync < 160) {
    		ReceivePacket(SX);

    		if (wirelessFeedback && !isWirelessTransmitting) {
    			// feedback enabled, transmit a packet to basestation
    			isWirelessTransmitting = true;
    			setChannel(SX, WIRELESS_FEEDBACK_CHANNEL); // set to channel 40 for feedback to basestation
    			SX->SX_settings->syncWords[0] = robot_syncWord[16]; // 0x82108610 for basestation Rx
    			setSyncWords(SX, SX->SX_settings->syncWords[0], 0x00, 0x00);
    			SendPacket(SX, data, Nbytes);
    		} else {
    			// feedback disabled, just stay in Rx mode
    			setRX(SX, SX->SX_settings->periodBase, WIRELESS_RX_COUNT);
    		}

    	}else{
    		// not necessary to force setRX() here when configured in Rx Continuous mode
    		setRX(SX, SX->SX_settings->periodBase, WIRELESS_RX_COUNT);
    	}
    }

    if(irq & SYNCWORD_VALID) {
    }

    if(irq & SYNCWORD_ERROR) {
    }

    /* Timeout is triggered after WIRELESS_RX_COUNT * 62.5 microseconds. Should be 4000 * 62.5 = 250ms */
    if(irq & RXTX_TIMEOUT) {
    	isWirelessConnected = false;
    	setRX(SX, SX->SX_settings->periodBase, WIRELESS_RX_COUNT);
    }

    if(irq & PREAMBLE_DETECTED) {
    }
};

void Wireless_DMA_Handler(SX1280* SX, uint8_t output[]){
	DMA_Callback(SX);
	if (SX->expect_packet) {
		// was expecting a packet, process it
		SX->expect_packet = false;
		memcpy(output, SX->RXbuf+3, SX->payloadLength);
	} else {
		// was not expecting a packet, go to Rx
		setRX(SX, SX->SX_settings->periodBase, WIRELESS_RX_COUNT);
	}
}
