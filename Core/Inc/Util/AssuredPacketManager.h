#ifndef __ASSURED_PACKET_MANAGER_H
#define __ASSURED_PACKET_MANAGER_H

/**
 * @file AssuredPacketManager.h
 * @author Emiel Steerneman
 * @date 2022-01-28
 */

/** @brief The AssuredPacketManager handles transmitting, retransmittion, and receiving ACKs
 * 
 * There should always be only one AssuredPacketManager (APM) struct! 
 * An APM keeps track of the sequence number by itself. Two APM's would track two different 
 * sequence number. If an AssuredAck packet then comes in, it will be unclear to which APM 
 * this AssuredAck belongs to.. 
 * 
 * Currently, the APM is limited to having only a single AssuredPacket in transport. Having more
 * packets in transport would be possible if multiple sequence numbers are tracked. However, this
 * requires arrays, more buffers, and what not, and I don't want to build that right now. The 
 * benefits would be minimal. The simplest way to extend the APM with this functionality would be
 * to create multiple instances, and give each instance their own range of sequence number. This
 * would assure that any incoming AssuredAck will always be absorbed by the correct APM instance.
 * 
 * To send an AssuredPacket, take the following steps:
 * 1. Check if the APM is in the READY state. If not, there is already another AssuredPacket waiting to be sent
 * 2. Give the message to the APM using the APM_sendAssuredMessage function. The APM will prepend it with an
 *    AssuredPacket header and sent it out as soon as possible.
 * 3. Give any received AssuredAck to the APM. The APM wil retransmit the AssuredPacket every N times until an
 *    AssuredAck with the correct sequence number has been received.
 * 4. Wait for the APM to go back to the READY state. You can now be assured that the packet has been received
 *    on the other side. The next AssuredPacket can be sent.
 **/

#include "REM_BaseTypes.h"
#include "REM_RobotAssuredPacket.h"
#include "REM_RobotAssuredAck.h"

/**
 * @brief Three states to keep track of the state of an AssuredPacket message transmitted by the robot
 * READY : There is no AssuredPacket in transport, and such a packet can be sent
 * AWAITING_TRANSMISSION : There is an AssuredPacket in the buffer, which is ready to be sent
 * AWAITING_ACK : An AssuredPacket has been sent, and the APM is now waiting for an AssuredAck
 *
 * State machine table
 *                              message given              valid ACK given      retransmission timeout exceeded
 *  READY                 :     AWAITING_TRANSMISSION      ACK ignored          nothing happens
 *  AWAITING_TRANSMISSION :     message ignored            ACK ignored          nothing happens
 *  AWAITING_ACK          :     message ignored            READY                AWAITING_TRANSMISSION
 */
typedef enum {
    READY,
    AWAITING_TRANSMISSION,
    AWAITING_ACK
} ASSURED_STATE ;

/**
 * @brief Struct to track the state, sequence number, and current message
 * 
 * @param state The state tracks if the APM is either READY, AWAITING_TRANSMISSION, or AWAITING_ACK.
 * @param transmission_timestamp The timestamp in milliseconds of the last transmission attempts. If the time 
 *  between this timestamp and the current timestamp exceeds RETRANSMISSION_DELAY_MS, the packet will be marked
 *  for retransmission by switching the APM to the state AWAITING_TRANSMISSION
 * @param sequence_number The sequence number tracks the "unique" (wraps around after 255) id of the AssuredPacket.
 *  Technically, this number is not needed right now since the APM currently only supports
 *  one AssuredPacket in transit at the time, but it good to have nonetheless I think.
 * @param message_length The size of the message in the buffer, EXCLUDING the size of the AssuredPacket header
 * @param message_buffer The buffer holds the AssuredPacket header, and the data to be sent
 */
typedef struct _AssuredPacketManager {
    ASSURED_STATE state;
    uint32_t transmission_timestamp;
    uint32_t sequence_number;
    uint8_t message_length;
    uint8_t message_buffer[127];
} AssuredPacketManager;

bool APM_isReady(AssuredPacketManager* apm);

bool APM_isAwaitingTransmission(AssuredPacketManager* apm);

bool APM_isAwaitingAck(AssuredPacketManager* apm);

/**
 * @brief Give the APM a message to send. APM needs to be in state READY
 * 
 * @param apm Pointer to the AssuredPacketManager instance
 * @param message Pointer to the message of maximum size 127 - PACKET_SIZE_ROBOT_ASSURED_PACKET
 * @param length length of the message
 * @return true Indicates that the packet will be sent
 * @return false Indicates that the message will not be sent
 */
bool APM_sendAssuredPacket(AssuredPacketManager* apm, uint8_t* message, uint8_t length);

/**
 * @brief Update the APM that the packet has been sent. APM will store the timestamp and go into the AWAITING_ACK state
 * 
 * @param apm Pointer to the AssuredPacketManager instance
 */
void APM_packetIsSent(AssuredPacketManager* apm);

/**
 * @brief Process a received AssuredAck. If valid, the APM will go into the READY state
 * 
 * @param apm Pointer to the AssuredPacketManager instance
 * @param raap Pointer to the RobotAssuredAckPayload instance
 */
void APM_absorbAssuredAck(AssuredPacketManager* apm, REM_RobotAssuredAckPayload* raap);

#endif /*__ASSURED_PACKET_MANAGER*/