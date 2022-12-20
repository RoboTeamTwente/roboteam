
#ifndef __SX1280_PACKET_DEFINITIONS_H
#define __SX1280_PACKET_DEFINITIONS_H

#include "SX1280_Constants.h"

// SX1280 Status Packets
/* Status byte layout - Table 11-5: Status Byte Definition, page 73 */
typedef struct __packed _SX1280_Status{
	uint8_t busy:1;
	uint8_t Reserved:1;
	SX1280_CommandStatus CommandStatus:3;
	SX1280_State InternalState:3;
}SX1280_Status;

/* Packet Status layout - Table 11-65: packetStatus Definition, page 93 */
typedef union {
    struct {
        uint8_t RFU:8;		                // reserved for future use (does nothing)
        uint8_t RSSISync:8;	                // signal power = -RSSISync/2 (dBm)
        uint8_t errors:8;	                // bit flags specified in type SX1280_Packet_Errors
        uint8_t status:8;	                // bit5: rxNoAck (dynamic payload), bit0: packet sent
        SX1280_Packet_Syncword sync:8;      // bit 2:0 : 000 syncword detection error, 001 syncword1 detected, 010 syncword2 detected, 100 syncword3 detected
    };
    uint8_t bytes[5];
}SX1280_Packet_Status;


// SX1280 ModParam struct
/* Modulation Parameters, page: 121 */
typedef union {
    struct {
        SX1280_BitRate bitrate:8;
        SX1280_CodingRate codingrate:8;
        SX1280_GaussFilter gaussfilter:8;
    };
    uint8_t bytes[3];
} SX1280_ModulationParam;

// SX1280 PacketParam Struct
/* Packet Parameters, page 122 */
typedef union {
    struct {
        SX1280_PreambleLength preamble:8;
        SX1280_SyncWordEnable syncenable:8;
        SX1280_MatchSyncWord matchsyncword:8;
        SX1280_PacketLengthType packetlengthtype:8;
        uint8_t payloadsize:8;
        SX1280_CRCLength crclength:8;
        uint8_t whitening:8;
    };
    uint8_t bytes[7];
} SX1280_PacketParam;

/* 11.8.1 GetRxBufferStatus, page 92 */
typedef union {
    struct {
        uint8_t payloadLength:8;
        uint8_t bufferOffset:8;
    };
    uint8_t bytes[2];
} SX1280_RX_Buffer_Status;

#endif //__SX1280_PACKET_DEFINITIONS_H