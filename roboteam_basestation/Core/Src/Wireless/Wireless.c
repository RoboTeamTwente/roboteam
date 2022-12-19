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

// Private Functions

// Checks if last command was processed succesfully
Wireless_Error validateCommand(Wireless* w);


// Sync Words for RobotID 'abcde'
/*
abcde	0abc deab cdea bcde		abcd e1ab cdea bcde		hex
00000	0000 0000 0000 0000 	0000 0100 0000 0000		0000 0400
00001	0000 0100 0010 0001		0000 1100 0010 0001		0421 0C21
00010	0000 1000 0100 0010		0001 0100 0100 0010		0842 1442
00011	0000 1100 0110 0011		0001 1100 0110 0011		0CC3 1CC3
00100	0001 0000 1000 0100		0010 0100 1000 0100		1084 2484
00101	0001 0100 1010 0101		0010 1100 1010 0101		14A5 2CA5
00110	0001 1000 1100 0110		0011 0100 1100 0110		18C6 34C6
00111	0001 1100 1110 0111		0011 1100 1110 0111		1CD7 3CD7
01000	0010 0001 0000 1000		0100 0101 0000 1000		2108 4508
01001	0010 0101 0010 1001		0100 1101 0010 1001		2529 4D29
01010	0010 1001 0100 1010		0101 0101 0100 1010		298A 554A
01011	0010 1101 0110 1011		0101 1101 0110 1011		2D6D 5D6D
01100	0011 0001 1000 1100		0110 0101 1000 1100		318C 658C
01101	0011 0101 1010 1101		0110 1101 1010 1101		35AD 6DAD
01110	0011 1001 1100 1110		0111 0101 1100 1110		39CE 75CE
01111	0011 1101 1110 1111		0111 1101 1110 1111		3DEF 7DEF

abcde	0abc deab cdea bcde		abcd e1ab cdea bcde		hex
10000	0100 0010 0001 0000 	1000 0110 0001 0000		8210 8610
10001	0100 0110 0011 0001		1000 1110 0011 0001		8631 8E31
10010	0100 1010 0101 0010		1001 0110 0101 0010		8A52 9652
10011	0100 1110 0111 0011		1001 1110 0111 0011		8ED3 9ED3
10100	0101 0010 1001 0100		1010 0110 1001 0100		9294 A694
10101	0101 0110 1011 0101		1010 1110 1011 0101		96B5 AEB5
10110	0101 1010 1101 0110		1011 0110 1101 0110		9AD6 B6D6
10111	0101 1110 1111 0111		1011 1110 1111 0111		9EE7 BEE7
11000	0110 0011 0001 1000		1100 0111 0001 1000		B318 C718
11001	0110 0111 0011 1001		1100 1111 0011 1001		B739 CF39
11010	0110 1011 0101 1010		1101 0111 0101 1010		BB9A D75A
11011	0110 1111 0111 1011		1101 1111 0111 1011		BF7D DF7D
11100	0111 0011 1001 1100		1110 0111 1001 1100		C39C E79C
11101	0111 0111 1011 1101		1110 1111 1011 1101		C7BD EFBD
11110	0111 1011 1101 1110		1111 0111 1101 1110		CBDE F7DE
11111	0111 1111 1111 1111		1111 1111 1111 1111		CFFF FFFF
*/
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

volatile static WIRELESS_CHANNEL currentChannel = YELLOW_CHANNEL;

// init structs
const SX1280_Settings SX1280_DEFAULT_SETTINGS = {
        .TXpower = 31, // -18 + txPower = transmit power in dBm (13dBm max)
		.packettype = PACKET_TYPE_FLRC,
        .TXrampTime = RADIO_RAMP_20_US,
		
        /* 11.6.4 SetTx, page 79. Time-out duration = periodBase * periodBaseCount. */
        /* 62.5 μs * 24 = 1.5ms timeout, which gives 666.6Hz. Enough for 11 robots at 60Hz each  */
        .periodBase = BASE_62_us,
        .periodBaseCount = 24,
		
		.syncWordTolerance = 2, // accepted wrong bits in a detected syncword
        .syncSensitivity = 1, // high sensitivity mode
        .crcSeed = 0xACB6, // seed value of 0xACB6 = 0b'1010110010110110
        .crcPoly = 0x1021, // poly of P16(x) = x16 + x12 + x5 + 1
        
        /* 8.4 Using the Data buffer. SX1280 has a 256 byte buffer. Give half to RX, half to TX */
        .TXoffset = 0x80,
        .RXoffset = 0x00,
        
        /* 14.3.1.5, page 121 */ .ModParam = {FLRC_BR_1_300_BW_1_2, FLRC_CR_3_4, BT_0_5}, /* Full power 1.3Mbps, 3/4 encoding rate, Pulse Shaping (Raised Cosine Filter) of 0.5 */ 
        /* 14.3.1.6, page 122 */ .PacketParam = {PREAMBLE_LENGTH_24_BITS, FLRC_SYNC_WORD_LEN_P32S, RX_MATCH_SYNC_WORD_1, PACKET_VARIABLE_LENGTH, MAX_PAYLOAD_SIZE, CRC_2_BYTE, NO_WHITENING},
        .DIOIRQ = {(IRQ_TX_DONE|IRQ_RX_DONE|IRQ_CRC_ERROR|IRQ_RXTX_TIMEOUT), (IRQ_TX_DONE|IRQ_RX_DONE|IRQ_CRC_ERROR|IRQ_RXTX_TIMEOUT), IRQ_NONE, IRQ_NONE}
};

// init functions
Wireless_Error Wireless_Init(Wireless* w, SX1280_Settings set, SX1280_Interface* interface){
	// Check inputs
    if (w==NULL || interface==NULL){
        return WIRELESS_PARAM_ERROR;
    }
    // fill Wireless struct with initial values
    w->Settings = set;
    w->Interface = interface;
    w->Interface->active_transfer = false;
    w->state = WIRELESS_INIT;
    w->TXSyncword = 0x0;
    w->RXSyncwords[0] = 0x0;
    w->RXSyncwords[1] = 0x0;
    w->readbufdest = NULL;
    w->readbufBytes = 0;
    w->TXchannel = WIRELESS_CHANNEL_DEFAULT_ROBOT_TO_BASESTATION;
    w->RXchannel = WIRELESS_CHANNEL_DEFAULT_BASESTATION_TO_ROBOT;
    
    // Start SX1280

    /* 14.3 FLRC Operation, page 121 */
	SX1280WakeUp(w->Interface); // reset, initialize SPI, set to STDBY_RC in order to configure SX1280

    /* 6.1 Overview, page 34. Note: Care must therefore be taken to ensure that modulation parameters are set
    using the command SetModulationParam() only after defining the packet type SetPacketType() to be used. */
    setPacketType(w->Interface, w->Settings.packettype); // packet type is set first!
    if(validateCommand(w) == WIRELESS_ERROR) return WIRELESS_ERROR;

    setChannel(w->Interface, w->RXchannel); // calls setRFFrequency() with freq=(channel+2400)*1000000
    if(validateCommand(w) == WIRELESS_ERROR) return WIRELESS_ERROR;

    setBufferBase(interface, w->Settings.TXoffset, w->Settings.RXoffset); // set offsets to write TX and RX data in the SX1280 buffer
    if(validateCommand(w) == WIRELESS_ERROR) return WIRELESS_ERROR;

    setModulationParam(w->Interface, &w->Settings.ModParam); // flrc = bitrate&BW, coding rate, modulation
    if(validateCommand(w) == WIRELESS_ERROR) return WIRELESS_ERROR;

    setPacketParam(w->Interface, &w->Settings.PacketParam); // flrc = preamble length, syncword length, syncword match, header, payload length, crc length, whitening=no
    if(validateCommand(w) == WIRELESS_ERROR) return WIRELESS_ERROR;

    setSyncSensitivity(w->Interface, w->Settings.syncSensitivity); // enable/disable highest gain step for receive
    if(validateCommand(w) == WIRELESS_ERROR) return WIRELESS_ERROR;

    setSyncWordTolerance(w->Interface, w->Settings.syncWordTolerance); // set how many bits can go wrong in the sync word
    if(validateCommand(w) == WIRELESS_ERROR) return WIRELESS_ERROR;

    setCrcSeed(w->Interface, w->Settings.crcSeed); // set crc seed value
    if(validateCommand(w) == WIRELESS_ERROR) return WIRELESS_ERROR;

    setCrcPoly(w->Interface, w->Settings.crcPoly); // set crc polynomial
    if(validateCommand(w) == WIRELESS_ERROR) return WIRELESS_ERROR;

    setTXParam(w->Interface, w->Settings.TXpower, w->Settings.TXrampTime); // set TX power and ramp up time
    if(validateCommand(w) == WIRELESS_ERROR) return WIRELESS_ERROR;

    setDIOIRQParams(w->Interface, w->Settings.DIOIRQ); // IRQ masks are stored in reverse, so reverse again
    if(validateCommand(w) == WIRELESS_ERROR) return WIRELESS_ERROR;

    // AutoFS reduces time between consecutive read / write operations
    setAutoFS(w->Interface, true); // to go or not to go FS after TX or RX
    if(validateCommand(w) == WIRELESS_ERROR) return WIRELESS_ERROR;
    
    w->state = WIRELESS_READY;
    return WIRELESS_OK;
};

Wireless_Error Wireless_setPrint_Callback(Wireless* w, Wireless_printf* func){
    // assume input is correct. There is no way to check, other than a compile error
    w->printf = func;
    if(w->printf) w->printf("[Wireless.c][Wireless_setPrint_Callback] Logging enabled\n");
    return WIRELESS_OK;
}

Wireless_Error Wireless_setIRQ_Callbacks(Wireless* w, Wireless_IRQcallbacks* irqcallbacks){
    // assume input is correct. There is no way to check, other than a compile error
    w->irqcallbacks = irqcallbacks;
    return WIRELESS_OK;
}

// runtime settings functions
Wireless_Error Wireless_setChannel(Wireless* w, WIRELESS_CHANNEL channel){
    switch (channel){
    case YELLOW_CHANNEL:
        w->TXchannel = WIRELESS_CHANNEL_YELLOW_BASESTATION_TO_ROBOT;
        w->RXchannel = WIRELESS_CHANNEL_YELLOW_ROBOT_TO_BASESTATION;
        break;
    case BLUE_CHANNEL:
        w->TXchannel = WIRELESS_CHANNEL_BLUE_BASESTATION_TO_ROBOT;
        w->RXchannel = WIRELESS_CHANNEL_BLUE_ROBOT_TO_BASESTATION;
        break;
    default:
        w->TXchannel = WIRELESS_CHANNEL_YELLOW_BASESTATION_TO_ROBOT;
        w->RXchannel = WIRELESS_CHANNEL_YELLOW_ROBOT_TO_BASESTATION;
        break;
    }
    // If the robot was in receiving mode, switch to the new frequency and continue receiving
    if(w->state == WIRELESS_RECEIVING){
        setChannel(w->Interface, w->RXchannel);
        if(w->continuousreceive){
            WaitForPacketContinuous(w);
        }else{
            WaitForPacket(w);
        }
    }
    return WIRELESS_OK;
}
WIRELESS_CHANNEL Wireless_getChannel(Wireless* w){
    if(w->TXchannel == WIRELESS_CHANNEL_YELLOW_BASESTATION_TO_ROBOT){
        return YELLOW_CHANNEL;
    }else{
        return BLUE_CHANNEL;
    }
    return BLUE_CHANNEL;
}

Wireless_Error Wireless_setTXSyncword(Wireless* w, uint32_t syncword){
    w->TXSyncword = syncword;
    if(setSyncWord(w->Interface, 1, syncword) != SX1280_OK){
        return WIRELESS_ERROR;
    }
    return WIRELESS_OK;
}

Wireless_Error Wireless_setRXSyncwords(Wireless* w, uint32_t syncwords[2]){
    // Determine which syncword channels should be used
    bool useSync0 = syncwords[0] != 0;
    bool useSync1 = syncwords[1] != 0;

    // look-up new syncword mask
    SX1280_MatchSyncWord newSyncwordMask = RX_DISABLE_SYNC_WORD;
    if(useSync0 && useSync1){
        newSyncwordMask = RX_MATCH_SYNC_WORD_2_3;
    } else if(useSync0 && !useSync1){
        newSyncwordMask = RX_MATCH_SYNC_WORD_2;
    } else if(!useSync0 && useSync1){
        newSyncwordMask = RX_MATCH_SYNC_WORD_3;
    }
    
    // Set new syncwords, only to register write if needed
    if(useSync0 && w->RXSyncwords[0] != syncwords[0]){
        w->RXSyncwords[0] = syncwords[0];
        if(setSyncWord(w->Interface, 2, syncwords[0]) != SX1280_OK){
            return WIRELESS_ERROR;
        }
    }
    if(useSync1 && w->RXSyncwords[1] != syncwords[1]){
        w->RXSyncwords[1] = syncwords[1];
        if(setSyncWord(w->Interface, 3, syncwords[1]) != SX1280_OK){
            return WIRELESS_ERROR;
        }
    }
    // Update packetparam if the newSyncwordMask is different from the current matchsyncword
    if(newSyncwordMask != w->Settings.PacketParam.matchsyncword){
        w->Settings.PacketParam.matchsyncword = newSyncwordMask;
        setPacketParam(w->Interface, &w->Settings.PacketParam);
    }
    return WIRELESS_OK;
}

// SX1280 buffer interface functions
Wireless_Error WritePacket(Wireless* w, Wireless_Packet* packet){
    if(w->state != WIRELESS_READY)
        return WIRELESS_ERROR;

    w->state = WIRELESS_WRITING;

    clearIRQ(w->Interface,IRQ_ALL);
    // If the packet that we're sending has a different size than the previous packet, we have to update the packet size in the SX1280
    // Table 14-38: Payload Length Definition in FLRC Packet, page 124
    if(w->Settings.PacketParam.payloadsize != packet->payloadLength){
        w->Settings.PacketParam.payloadsize = packet->payloadLength;
        setPacketParam(w->Interface, &w->Settings.PacketParam);
    }
    writeBuffer(w->Interface, w->Settings.TXoffset, packet->message, packet->payloadLength);
    w->state = WIRELESS_READY;
    return WIRELESS_OK;
}
Wireless_Error WritePacket_DMA(Wireless* w, Wireless_Packet* packet, Wireless_Writepacket_Callback* func){
    if(w->state != WIRELESS_READY)
        return WIRELESS_ERROR;
         
    w->state = WIRELESS_WRITING;
    
    w->wrcallback = func;

    clearIRQ(w->Interface,IRQ_ALL);
    // If the packet that we're sending has a different size than the previous packet, we have to update the packet size in the SX1280
    // Table 14-38: Payload Length Definition in FLRC Packet, page 124
    if(w->Settings.PacketParam.payloadsize != packet->payloadLength){
        w->Settings.PacketParam.payloadsize = packet->payloadLength;
        setPacketParam(w->Interface, &w->Settings.PacketParam);
    }
    writeBuffer_DMA(w->Interface, w->Settings.TXoffset, packet->message, packet->payloadLength);
    return WIRELESS_OK;
}

Wireless_Error ReadPacket(Wireless* w, Wireless_Packet* packet){
    w->state = WIRELESS_READING;

    clearIRQ(w->Interface,IRQ_ALL);
    SX1280_RX_Buffer_Status bs;
    getRXBufferStatus(w->Interface, &bs);
    readBuffer(w->Interface, bs.bufferOffset, packet->message, bs.payloadLength);
    packet->payloadLength = bs.payloadLength;

    w->state = w->continuousreceive ? WIRELESS_RECEIVING : WIRELESS_READY;
    return WIRELESS_OK;
};

Wireless_Error ReadPacket_DMA(Wireless* w, Wireless_Packet* packet, Wireless_Readpacket_Callback* func){
    w->state = WIRELESS_READING;
    w->readbufdest = packet->message;
    w->rdcallback = func;

    clearIRQ(w->Interface,IRQ_ALL);
    SX1280_RX_Buffer_Status bs;
    getRXBufferStatus(w->Interface, &bs);
    readBuffer_DMA(w->Interface, bs.bufferOffset, packet->message, bs.payloadLength);
    packet->payloadLength = bs.payloadLength;
    w->readbufBytes = packet->payloadLength;
    return WIRELESS_OK;
};

// Send/Receive functions
// write to buffer before calling TransmitPacket, otherwise the previous packet will be send
Wireless_Error TransmitPacket(Wireless* w){
    w->state = WIRELESS_TRANSMITTING;
    w->Settings.PacketParam.matchsyncword = RX_MATCH_SYNC_WORD_1;
    setPacketParam(w->Interface, &w->Settings.PacketParam);  
    setChannel(w->Interface, w->TXchannel);
    setTX(w->Interface, w->Settings.periodBase, w->Settings.periodBaseCount);
    return WIRELESS_OK;
}
Wireless_Error WaitForPacket(Wireless* w){    
    w->state = WIRELESS_RECEIVING;
    w->continuousreceive = false;
    w->Settings.PacketParam.matchsyncword = RX_MATCH_SYNC_WORD_2_3;
    setPacketParam(w->Interface, &w->Settings.PacketParam);  
    setChannel(w->Interface, w->RXchannel);
    setRX(w->Interface, w->Settings.periodBase, w->Settings.periodBaseCount);
    return WIRELESS_OK;
}
Wireless_Error WaitForPacketContinuous(Wireless* w){
    w->state = WIRELESS_RECEIVING;
    w->continuousreceive = true;
    setChannel(w->Interface, w->RXchannel);
    setRX(w->Interface, w->Settings.periodBase, 0xFFFF);
    return WIRELESS_OK;
}

// -------------------------------------------- Handlers
Wireless_Error Wireless_IRQ_Handler(Wireless* w){
    uint16_t irq;
    SX1280_Packet_Status ps;
    getIRQ(w->Interface, &irq);
    
    clearIRQ(w->Interface,IRQ_ALL);

    bool callback_handled = false;

    if(irq & IRQ_CRC_ERROR) {
        if(w->irqcallbacks && w->irqcallbacks->crcerror){
            if(w->printf) w->printf("[Wireless_IRQ_Handler] IRQ_CRC_ERROR\n");
            w->irqcallbacks->crcerror();
            callback_handled = true;
        }
    }

    /* Packet sent to robot */
    if(irq & IRQ_TX_DONE){
        if(w->state == WIRELESS_TRANSMITTING) w->state = WIRELESS_READY;
        getPacketStatus(w->Interface, &ps);
        if(w->irqcallbacks && w->irqcallbacks->txdone){
            if(w->printf) w->printf("[Wireless_IRQ_Handler] IRQ_TX_DONE\n");
            w->irqcallbacks->txdone(&ps);
            callback_handled = true;
        }
    }

    // Note : IRQ_RX_DONE also triggers when there is an crc error, so ignore IRQ_RX_DONE when IRQ_CRC_ERROR is also set: 16.3 All Modems: Interrupt with Bad CRC, page 150
    if(irq & IRQ_RX_DONE && !(irq & IRQ_CRC_ERROR)){
        getPacketStatus(w->Interface, &ps);
        if(w->irqcallbacks && w->irqcallbacks->rxdone){
            if(w->printf) w->printf("[Wireless_IRQ_Handler] IRQ_RX_DONE\n");
            w->irqcallbacks->rxdone(&ps);
            callback_handled = true;
        }
    }

    if(irq & IRQ_RXTX_TIMEOUT) {
        if(w->irqcallbacks && w->irqcallbacks->rxtxtimeout){
            if(w->printf) w->printf("[Wireless_IRQ_Handler] IRQ_RXTX_TIMEOUT\n");
            w->irqcallbacks->rxtxtimeout();
            callback_handled = true;
        }
    }
    
    if(irq & IRQ_SYNCWORD_VALID) {
        if(w->irqcallbacks && w->irqcallbacks->syncvalid){
            if(w->printf) w->printf("[Wireless_IRQ_Handler] IRQ_SYNCWORD_VALID\n");
            w->irqcallbacks->syncvalid();
            callback_handled = true;
        }
    }

    if(irq & IRQ_SYNCWORD_ERROR) {
        if(w->irqcallbacks && w->irqcallbacks->syncerror){
            if(w->printf) w->printf("[Wireless_IRQ_Handler] IRQ_SYNCWORD_ERROR\n");
            w->irqcallbacks->syncerror();
            callback_handled = true;
        }
    }

    if(irq & IRQ_PREAMBLE_DETECTED) {
        if(w->irqcallbacks && w->irqcallbacks->preambledetected){
            if(w->printf) w->printf("[Wireless_IRQ_Handler] IRQ_PREAMBLE_DETECTED\n");
            w->irqcallbacks->preambledetected();
            callback_handled = true;
        }
    }

    if(!callback_handled){
        if(w->irqcallbacks && w->irqcallbacks->default_callback){
            if(w->printf) w->printf("[Wireless_IRQ_Handler] Default IRQ Handler for %d\n", irq);
            w->irqcallbacks->default_callback();
        }else{
            /* WARNING! An IRQ has not been handled properly. This should never happen */
            if(w->printf){
                w->printf("[Wireless.c][Wireless_IRQ_Handler] Warning : interrupt not handled!\n");
            }else{
                /* Unsure what to do here? How to let people know that an interrupt has not been handled? */
            }
        }
    }

    return WIRELESS_OK;
};

Wireless_Error Wireless_DMA_Handler(Wireless* w){
    switch (w->state){
    case WIRELESS_READING:
        DMA_Callback(w->Interface, w->readbufdest, w->readbufBytes);
        if(w->continuousreceive){
            w->state = WIRELESS_RECEIVING;
        } else{
            w->state = WIRELESS_READY;
        }
        if(w->rdcallback){
            w->rdcallback();
            w->rdcallback = NULL;
        }
        return WIRELESS_OK;

    case WIRELESS_WRITING:
        DMA_Callback(w->Interface, NULL, 0);
        w->state = WIRELESS_READY;
        if(w->wrcallback){
            w->wrcallback();
            w->wrcallback = NULL;
        }
        return WIRELESS_OK;

    default:
        if(w->printf){
            w->printf("Wireless DMA handler called while in the wrong state");
        }
        return WIRELESS_ERROR;
    }
}


// Private Functions
Wireless_Error validateCommand(Wireless* w){
    // Get the status of the SX after the last transaction to see if the command was processed correctly
    SX1280_CommandStatus stat = getStatus(w->Interface).CommandStatus;
    // C
    switch (stat)
    {
    case Success:
        return WIRELESS_OK;
    case DataAvailable:
        return WIRELESS_OK;
    case TimeOut:
        if(w->printf){
            w->printf("ERROR: SX command TimeOut");
        }
        return WIRELESS_ERROR;
	case ProcessingErr:
        if(w->printf){
            w->printf("ERROR: SX command ProcessingErr");
        }
        return WIRELESS_ERROR;
	case Failed:
        if(w->printf){
            w->printf("ERROR: SX command Failed");
        }
        return WIRELESS_ERROR;
	case TxDone:
        return WIRELESS_OK;
    default:
        break;
    }
    return WIRELESS_OK;
}