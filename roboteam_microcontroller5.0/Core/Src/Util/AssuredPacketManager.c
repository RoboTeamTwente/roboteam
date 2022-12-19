#include <stdint.h>
#include <stdbool.h>
#include <string.h>

#include "main.h"
#include "robot.h"

#include "REM_BaseTypes.h"
#include "AssuredPacketManager.h"

// The time needed in milliseconds before a packet is being retransmitted
static const uint32_t RETRANSMISSION_DELAY_MS = 3000;

bool APM_isReady(AssuredPacketManager* apm){
    return apm->state == READY;
}

bool APM_isAwaitingTransmission(AssuredPacketManager* apm){
    // Check if an AssuredAck is expected and if the RETRANSMISSION_DELAY_MS is exceeded
    // If so, move state to AWAITING_TRANSMISSION
    if(APM_isAwaitingAck(apm)){
        uint32_t current_time = HAL_GetTick();
        if(RETRANSMISSION_DELAY_MS < current_time - apm->transmission_timestamp)
            apm->state = AWAITING_TRANSMISSION;
    }

    return apm->state == AWAITING_TRANSMISSION;
}

bool APM_isAwaitingAck(AssuredPacketManager* apm){
    return apm->state == AWAITING_ACK;
}

bool APM_sendAssuredPacket(AssuredPacketManager* apm, uint8_t* message, uint8_t length){
    // An AssuredPacket can only be sent if another one is not already in transit
    if(!APM_isReady(apm))
        return false;

    // TODO replace the magic number 127 with a constant, reflecting the max SX1280 buffer size
    // Ensure that the packet can actually be transmitted by the SX1280. Yes, it might be possible
    // to send larger packets over UART, but please just split it up into a few smaller packets.
    if(127 < length + REM_PACKET_SIZE_REM_ROBOT_ASSURED_PACKET)
        return false;

    // Increase the sequence number
    apm->sequence_number++;

    // Create the AssuredPacket header
    REM_RobotAssuredPacketPayload rapp;
    REM_RobotAssuredPacket_set_header(&rapp, REM_PACKET_TYPE_REM_ROBOT_ASSURED_PACKET);
    REM_RobotAssuredPacket_set_remVersion(&rapp, REM_LOCAL_VERSION);
    REM_RobotAssuredPacket_set_id(&rapp, robot_get_ID());
    REM_RobotAssuredPacket_set_sequenceNumber(&rapp, apm->sequence_number);
    REM_RobotAssuredPacket_set_messageLength(&rapp, length);

    // Copy the AssuredPacket header into the message buffer
    memcpy(apm->message_buffer, rapp.payload, REM_PACKET_SIZE_REM_ROBOT_ASSURED_PACKET);
    // Copy the rest of the message into the message buffer
    memcpy(apm->message_buffer + REM_PACKET_SIZE_REM_ROBOT_ASSURED_PACKET, message, length);

    apm->message_length = length;
    apm->state = AWAITING_TRANSMISSION;

    return true;
}

void APM_packetIsSent(AssuredPacketManager* apm){
    apm->transmission_timestamp = HAL_GetTick();
    apm->state = AWAITING_ACK;    
}

void APM_absorbAssuredAck(AssuredPacketManager* apm, REM_RobotAssuredAckPayload* raap){
    // Ignore any AssuredAck if APM is not expecting one
    if(!APM_isAwaitingAck(apm))
        return;

    // Get the sequence number from the AssuredAck
    uint8_t ack_sequence_number = REM_RobotAssuredAck_get_sequenceNumber(raap);
    // Ensure the sequence number is correct
    if(ack_sequence_number != apm->sequence_number)
        return;
    
    // Reset message_length to 0, to prevent accidental retransmission
    apm->message_length = 0;
    // Ready to send a new AssuredPacket
    apm->state = READY;
}
