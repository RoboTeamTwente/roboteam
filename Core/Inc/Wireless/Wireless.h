/*
 * Wireless.h
 *
 *  Created on: 6 feb. 2019
 *      Author: Cas Doornkamp
 * 
 * Implements functionality for main to communicate with the SX1280
 */

#ifndef WIRELESS_WIRELESS_H_
#define WIRELESS_WIRELESS_H_

#include <stdbool.h>
#include <stdint.h>
#include "SX1280_Constants.h"
#include "SX1280.h"
#include "main.h"

#define WIRELESS_YELLOW_CHANNELS 0
#define WIRELESS_BLUE_CHANNELS 1

#define WIRELESS_CHANNEL_YELLOW_ROBOT_TO_BASESTATION 30 // 2.43 GHz
#define WIRELESS_CHANNEL_YELLOW_BASESTATION_TO_ROBOT 40 // 2.44 GHz
#define WIRELESS_CHANNEL_BLUE_ROBOT_TO_BASESTATION 50 // 2.45 GHz
#define WIRELESS_CHANNEL_BLUE_BASESTATION_TO_ROBOT 60 // 2.46 GHz

#define WIRELESS_CHANNEL_DEFAULT_ROBOT_TO_BASESTATION WIRELESS_CHANNEL_YELLOW_ROBOT_TO_BASESTATION
#define WIRELESS_CHANNEL_DEFAULT_BASESTATION_TO_ROBOT WIRELESS_CHANNEL_YELLOW_BASESTATION_TO_ROBOT


typedef enum WIRELESS_CHANNEL {
    YELLOW_CHANNEL = 0,
    BLUE_CHANNEL = 1
} WIRELESS_CHANNEL;

extern uint32_t robot_syncWord[];

////////////////////////////////////// Function Types
typedef void Wireless_printf(char *format, ...);
// DMA complete callback functions
typedef void Wireless_Writepacket_Callback(void);
typedef void Wireless_Readpacket_Callback(void);
// IRQ callback functions
typedef void Wireless_TXDone_Callback(SX1280_Packet_Status *Packetstatus);
typedef void Wireless_RXDone_Callback(SX1280_Packet_Status *Packetstatus);
typedef void Wireless_SyncValid_Callback(void);
typedef void Wireless_SyncError_Callback(void);
typedef void Wireless_CRCError_Callback(void);
typedef void Wireless_RXTXTimeout_Callback(void);
typedef void Wireless_PreambleDetected_Callback(void);
typedef void Wireless_Default_Callback(void);

// TODO: Make a wireless error handler? In case an error occured, do this:??


////////////////////////////////////// Enums
typedef enum Wireless_Error{
    WIRELESS_OK,
    WIRELESS_ERROR,
    WIRELESS_PARAM_ERROR,
} Wireless_Error;

typedef enum Wireless_State{
    WIRELESS_RESET,
    WIRELESS_INIT,
    WIRELESS_READY,
    WIRELESS_WRITING,
    WIRELESS_READING,
    WIRELESS_TRANSMITTING,
    WIRELESS_RECEIVING,
} Wireless_State;

////////////////////////////////////// Structs
// Struct with callback functions for when an irq is triggered that is active (from the irqs in SX1280_IRQ)
// Note: The corresponding IRQ needs to be activated and written in SX1280_Settings for the function to be called
typedef struct Wireless_IRQcallbacks {
    Wireless_TXDone_Callback* txdone;
    Wireless_RXDone_Callback* rxdone;
    Wireless_SyncValid_Callback* syncvalid;
    Wireless_SyncError_Callback* syncerror;
    Wireless_CRCError_Callback* crcerror;
    Wireless_RXTXTimeout_Callback* rxtxtimeout;
    Wireless_PreambleDetected_Callback* preambledetected;
    Wireless_Default_Callback* default_callback;
} Wireless_IRQcallbacks;

typedef struct Wireless_Packet {
    volatile uint8_t message[MAX_PAYLOAD_SIZE];
    volatile uint8_t payloadLength;
} Wireless_Packet;


typedef struct Wireless {
    SX1280_Interface* Interface;                // Interface struct containing all outside connections needed to communicate with the SX
    uint32_t TXSyncword;                        // Stores the current TX syncword used for sending packets
    uint32_t RXSyncwords[2];                    // Stores the syncwords the SX listens to when receiving packets, if 0 the syncword is inactive
    int32_t TXchannel;  	                    // current channel used for sending
    int32_t RXchannel;                          // current channel used for receiving
    Wireless_State state;                       // Emulated internal state of the SX
    bool continuousreceive;                     // bool that saves if the SX is configured to keep receiving packets
    uint8_t* readbufdest;                       // pointer used when read buffer with dma has been called, TODO: maybe refactor writing/reading to buffer so this can be removed
    uint8_t readbufBytes;                       // amount of bytes used when read buffer with dma has been called, TODO: maybe refactor writing/reading to buffer so this can be removed
    Wireless_printf* printf;                    // callback function where to print debug messages to (if not NULL)
    Wireless_IRQcallbacks* irqcallbacks;        // struct of irq callback functions for the user to implement, only needed functionality has to be implemented
    Wireless_Writepacket_Callback* wrcallback;  // callback function pointer used when WritePacket_DMA has completed, reset to NULL after completion
    Wireless_Writepacket_Callback* rdcallback;  // callback function pointer used when ReadPacket_DMA has completed, reset to NULL after completion
    SX1280_Settings Settings;                   // private struct which holds the up-to-date settings of the sx1280
} Wireless;

////////////////////////////////////// Public Functions
// init functions
Wireless_Error Wireless_Init(Wireless* w, SX1280_Settings set, SX1280_Interface* interface);
Wireless_Error Wireless_setPrint_Callback(Wireless* w, Wireless_printf* func);
Wireless_Error Wireless_setIRQ_Callbacks(Wireless* w, Wireless_IRQcallbacks* irqcallbacks);

// runtime settings functions
Wireless_Error Wireless_setChannel(Wireless* w, WIRELESS_CHANNEL channel);
WIRELESS_CHANNEL Wireless_getChannel(Wireless* w);
Wireless_Error Wireless_setTXSyncword(Wireless* w, uint32_t syncword);
Wireless_Error Wireless_setRXSyncwords(Wireless* w, uint32_t syncwords[2]);

// SX1280 buffer interface functions
Wireless_Error WritePacket(Wireless* w, Wireless_Packet* packet);
Wireless_Error WritePacket_DMA(Wireless* w, Wireless_Packet* packet, Wireless_Writepacket_Callback* func);
Wireless_Error ReadPacket(Wireless* w, Wireless_Packet* packet);
Wireless_Error ReadPacket_DMA(Wireless* w, Wireless_Packet* packet, Wireless_Readpacket_Callback* func);

// Send/Receive functions
// write to buffer before calling TransmitPacket, otherwise the previous packet will be send
Wireless_Error TransmitPacket(Wireless* w);
Wireless_Error WaitForPacket(Wireless* w);
Wireless_Error WaitForPacketContinuous(Wireless* w);

// Callback functions for Hardware entry
Wireless_Error Wireless_IRQ_Handler(Wireless* w);
Wireless_Error Wireless_DMA_Handler(Wireless* w);

#endif /* WIRELESS_WIRELESS_H_ */
