#include "basestation.h"
#include "main.h"
#include "Wireless.h"
#include "TextOut.h"
#include "packet_buffers.h"
#include "FT812Q_Drawing.h"

#include "BaseTypes.h"
#include "BasestationStatistics.h"
#include "RobotCommand.h"
#include "RobotFeedback.h"
#include "RobotStateInfo.h"

volatile int handled_RobotCommand = 0;
volatile int handled_RobotFeedback = 0;
volatile int handled_RobotBuzzer = 0;
volatile int handled_RobotStateInfo = 0;

extern SPI_HandleTypeDef hspi1;
extern SPI_HandleTypeDef hspi2;
extern TIM_HandleTypeDef htim1;

/* Screen variables */
DISPLAY_STATES displayState = DISPLAY_STATE_DEINITIALIZED;
uint16_t touchPoint[2] = {-1, -1}; // Initialize touchPoint outside of screen, meaning TOUCH_STATE_RELEASED
TouchState touchState; // TODO check default initialization. What is touchState->state? Compiler dependent?
char logBuffer[100];

// Based on Wireless.c:SX1280_Settings.TXoffset
// Currently, we're splitting the SX1280 256 byte buffer in half. 128 for sending, 128 for receiving
// Set to 127, because that's the max value as defined in the datasheet
// Table 14-38: Payload Length Definition in FLRC Packet, page 124
#define MAX_PACKET_SIZE 127
uint8_t sendBuffer[MAX_PACKET_SIZE];

/* Flags */
volatile bool flagHandleStatistics = false;

void init(){
    HAL_Delay(1000); // TODO Why do we have this again? To allow for USB to start up iirc?
    
    LOG("[init] Initializing SX_TX\n");
    SX_TX = Wireless_Init(WIRELESS_COMMAND_CHANNEL, &hspi1, 0);
    
    LOG("[init] Initializing SX_RX\n");
    SX_RX = Wireless_Init(WIRELESS_FEEDBACK_CHANNEL, &hspi2, 1);

    // Set SX_RX syncword to basestation syncword
    SX_RX->SX_settings->syncWords[0] = robot_syncWord[16];
    setSyncWords(SX_RX, SX_RX->SX_settings->syncWords[0], 0x00, 0x00);
    // Start listening on the SX_RX
    setRX(SX_RX, SX_RX->SX_settings->periodBase, 0xFFFF);

    // Start the timer that is responsible for sending packets
    LOG("[init] Initializing Timer\n");
    HAL_TIM_Base_Start_IT(&htim1);

    // display_Init();
    // displayState = DISPLAY_STATE_INITIALIZED;
    // drawBasestation(true);

    LOG("[init] Initializion complete\n");
}

// screenCounter acts as a timer for updating the screen
uint32_t screenCounter = 0;

uint32_t heartbeat_1000ms = 0;

void loop(){
  
  /* Send log if any */
  if(0 < strlen(logBuffer)){
    LOG(logBuffer);
    logBuffer[0] = '\0';
  }

  /* Heartbeat every second */
  if(heartbeat_1000ms + 1000 < HAL_GetTick()){
    heartbeat_1000ms += 1000;
    sprintf(logBuffer, "Tick | RC %d RF %d RB %d RSI %d\n",
    handled_RobotCommand, handled_RobotFeedback, handled_RobotBuzzer, handled_RobotStateInfo);
    LOG(logBuffer);
    logBuffer[0] = '\0';
  }

  // TODO put multiple of these messages into a single USB packet, instead of sending every packet separately

  /* Send any new RobotFeedback packets */
  for(int id = 0; id < MAX_ROBOT_ID; id++){
    if(buffer_RobotFeedback[id].isNewPacket){
      HexOut(buffer_RobotFeedback[id].packet.payload, PACKET_SIZE_ROBOT_FEEDBACK);
      buffer_RobotFeedback[id].isNewPacket = false;
    }
  }

  /* Send any new RobotStateInfo packets */
  for(int id = 0; id < MAX_ROBOT_ID; id++){
    if(buffer_RobotStateInfo[id].isNewPacket){
      handled_RobotBuzzer++;
      HexOut(buffer_RobotStateInfo[id].packet.payload, PACKET_SIZE_ROBOT_STATE_INFO);
      buffer_RobotStateInfo[id].isNewPacket = false;
    }
  }

  if(flagHandleStatistics){
    handleStatistics();
    flagHandleStatistics = false;
  }


  /* Skip all screen stuff */
  return;


  /* Ensures that the CPU doesn't get overloaded with display stuff */
  // TODO switch to HAL_GetTick instead of counter
  if(screenCounter++ < 80000){
    LOG("Tick\n");
    return;
  }
    
  screenCounter = 0;


  if(displayState != DISPLAY_STATE_DEINITIALIZED)
    updateTouchState(&touchState);

  /* Do screen stuff */
  // TODO this should probably go somewhere else. Maybe to FT812Q.c?
  
  /* TODO redesign these states */
  switch(displayState){
    case DISPLAY_STATE_DEINITIALIZED:
      // TODO What to do here? Initialize? Do we want to be able to deinitialize the screen?
      break;
    case DISPLAY_STATE_INITIALIZED:
      displayState = DISPLAY_STATE_MAIN;
      break;
    case DISPLAY_STATE_MAIN:
      if(touchState.state == TOUCH_STATE_PRESSED)
        drawBrightnessSlider(&touchState);
      else
        drawBasestation(false);
      break;
    case DISPLAY_STATE_ROBOT:
      // TODO make 14 dynamic. Add touch events etc
      drawRobotInfo(14, false);
  }
}



/**
 * @brief Updates the state of the touch. This helps identifying
 * multiple touch evsents as a single screen press, keeps track on
 * if the screen press has been handled, and stores the location 
 * of the last touch event
 * TODO there might be a better place for this. Maybe in FT812Q.c?
 * @param touchState The touchState struct that has to be updated
 */
void updateTouchState(TouchState* touchState){
  // Read the coordinates of a (possible) touch
  readTouch(touchPoint);
  // Calculate if the user is currently touching the screen
  bool touching = 0 < touchPoint[0] && touchPoint[0] < XRES && 0 < touchPoint[1] && touchPoint[1] < YRES;
  /* If the screen is currently being pressed */
  if(touching){
    touchState->x = touchPoint[0];
    touchState->y = touchPoint[1];
    /* If the screen wasn't pressed before, this is a new press */
    if(touchState->state == TOUCH_STATE_RELEASED){
      touchState->state = TOUCH_STATE_PRESSED;  // Set the state to pressed
      touchState->handled = false;              // Set the event being handled to false
    }
  }
  /* If the screen is currently not being presed */
  else{
    /* If the screen was pressed before, it is now released */
    if(touchState->state == TOUCH_STATE_PRESSED){
      touchState->state = TOUCH_STATE_RELEASED;
    }
  }
}


/**
 * @brief Receives a buffer which is assumed to be holding a RobotCommand packet. 
 * If so, it moves the packet to the buffer, and sets a flag to send the packet to
 * the corresponding robot.
 * 
 * @param Buf Pointer to the buffer that holds the packet
 * @return true if the packet has been handled succesfully
 * @return false if there was something wrong with the packet
 */
bool handleRobotCommand(uint8_t* Buf){
  handled_RobotCommand++;
  // Interpret buffer as a RobotCommandPayload
  RobotCommandPayload *rcp = (RobotCommandPayload*) Buf;

  // Check if the robotId is valid
  uint8_t robotId = RobotCommand_get_id(rcp);
  if (MAX_ROBOT_ID < robotId)
    return false;
    
  // Store the message in the buffer. Set flag to be sent to the robot
  memcpy(buffer_RobotCommand[robotId].packet.payload, Buf, PACKET_SIZE_ROBOT_COMMAND);
  buffer_RobotCommand[robotId].isNewPacket = true;
  buffer_RobotCommand[robotId].counter++;
    
  return true;
}


/**
 * @brief Receives a buffer which is assumed to be holding a RobotFeedback packet. 
 * If so, it moves the packet to the buffer, and sets a flag to send the packet to
 * the computer.
 * 
 * @param Buf Pointer to the buffer that holds the packet
 * @return true if the packet has been handled succesfully
 * @return false if there was something wrong with the packet
 */
bool handleRobotFeedback(uint8_t* Buf){
  handled_RobotFeedback++;
  // Interpret buffer as a RobotFeedbackPayload
  RobotFeedbackPayload *rfp = (RobotFeedbackPayload*) Buf;

  // Check if the robotId is valid
  uint8_t robotId = RobotFeedback_get_id(rfp);
  if (MAX_ROBOT_ID < robotId)
    return false;
    
  // Store the message in the buffer. Set flag to be sent to the robot
  memcpy(buffer_RobotFeedback[robotId].packet.payload, Buf, PACKET_SIZE_ROBOT_FEEDBACK);
  buffer_RobotFeedback[robotId].isNewPacket = true;
  buffer_RobotFeedback[robotId].counter++;
    
  return true;
}


/**
 * @brief Receives a buffer which is assumed to be holding a handleRobotStateInfo packet. 
 * If so, it moves the packet to the buffer, and sets a flag to send the packet to
 * the computer.
 * 
 * @param Buf Pointer to the buffer that holds the packet
 * @return true if the packet has been handled succesfully
 * @return false if there was something wrong with the packet
 */
bool handleRobotStateInfo(uint8_t* Buf){
  handled_RobotStateInfo++;
  // Interpret buffer as a RobotStateInfoPayload
  RobotStateInfoPayload *rsip = (RobotStateInfoPayload*) Buf;

  // Check if the robotId is valid
  uint8_t robotId = RobotStateInfo_get_id(rsip);
  if (MAX_ROBOT_ID < robotId)
    return false;
    
  // Store the message in the buffer. Set flag to be sent to the robot
  memcpy(buffer_RobotStateInfo[robotId].packet.payload, Buf, PACKET_SIZE_ROBOT_STATE_INFO);
  buffer_RobotStateInfo[robotId].isNewPacket = true;
  buffer_RobotStateInfo[robotId].counter++;
    
  return true;
}


/**
 * @brief Handles sending basestation statistics over USB.
 * 
 * @return true if the packet has been handled succesfully
 * @return false if there was something wrong with the packet
 */
bool handleStatistics(void){
  // Not dealing with the BasestationStatistics struct. Due to autogenerating, it's not suited for for-loops
  BasestationStatisticsPayload bsp = {0};
  bsp.payload[0] = PACKET_TYPE_BASESTATION_STATISTICS;
  for(int robotId = 0; robotId < 16; robotId++){
    // Send ratio + packets sent (clipped) for each robot
    // TODO this clipping is not nice at all. Next to that, we need to find some way to figure out
    // how to draw if robots are "active", meaning we need to reset these stats every few ms or so.
    bsp.payload[robotId*2+1] = buffer_RobotCommand [robotId].counter >= 255 ? 255 : buffer_RobotCommand [robotId].counter;
    bsp.payload[robotId*2+2] = buffer_RobotFeedback[robotId].counter >= 255 ? 255 : buffer_RobotFeedback[robotId].counter;
    // Reset statistics
    buffer_RobotCommand [robotId].counter = 0;
    buffer_RobotFeedback[robotId].counter = 0;
  }
  HexOut(bsp.payload, PACKET_SIZE_BASESTATION_STATISTICS);
  return true;
}


/**
 * @brief Receives a buffer which is assumed to be holding a RobotCommand packet. 
 * If so, it moves the packet to the buffer, and sets a flag to send the packet to
 * the corresponding robot.
 * 
 * @param Buf Pointer to the buffer that holds the packet
 * @return true if the packet has been handled succesfully
 * @return false if there was something wrong with the packet
 */
bool handleRobotBuzzer(uint8_t* Buf){
  handled_RobotBuzzer++;
  // Interpret buffer as a RobotBuzzerPayload
  RobotBuzzerPayload *rbp = (RobotBuzzerPayload*) Buf;

  // Check if the robotId is valid
  uint8_t robotId = RobotBuzzer_get_id(rbp);
  if (MAX_ROBOT_ID < robotId)
    return false;
    
  // Store the message in the buffer. Set flag to be sent to the robot
  memcpy(buffer_RobotBuzzer[robotId].packet.payload, Buf, PACKET_SIZE_ROBOT_BUZZER);
  buffer_RobotBuzzer[robotId].isNewPacket = true;
  buffer_RobotBuzzer[robotId].counter++;
    
  return true;
}


/**
 * @brief routes any incoming packet to the correct function. Hub for all incoming packets.
 * TODO actually make it route all incoming packets, and not just USB packets
 * TODO add handling for undefined ./ unknown packets
 * 
 * Note : Packets are never larger than 64 bytes. If the host sends 100 bytes of data, it
 * will arrive as two packets of 64 and 36 bytes. This is because the wMaxPacketSize for
 * full speed USB is 64 bytes.
 * 
 * @param Buf Pointer to the buffer the packet from the USB is stored in
 * @param Len Length of the packet currently in the buffer
 * @return true if the packed has been handled succesfully
 * @return false if the packet has been handled unsuccessfully, e.g. due to corruption
 */
bool handlePacket(uint8_t* packet, uint32_t packet_length){
  uint8_t packet_type;
  uint32_t bytes_processed = 0;

  bool success = true;

  while(bytes_processed < packet_length){

    packet_type = packet[bytes_processed];

    switch (packet_type){

    case PACKET_TYPE_ROBOT_COMMAND:
      bytes_processed += PACKET_SIZE_ROBOT_COMMAND;
      success = handleRobotCommand(packet);
      break;
    
    case PACKET_TYPE_ROBOT_FEEDBACK:
      bytes_processed += PACKET_SIZE_ROBOT_FEEDBACK;
      success = handleRobotFeedback(packet);
      break;
    
    case PACKET_TYPE_BASESTATION_GET_STATISTICS:
      bytes_processed += PACKET_SIZE_BASESTATION_GET_STATISTICS;
      flagHandleStatistics = true;  
      break;
    
    case PACKET_TYPE_ROBOT_BUZZER:
      bytes_processed += PACKET_SIZE_ROBOT_BUZZER;
      success = handleRobotBuzzer(packet);
      break;

    case PACKET_TYPE_ROBOT_STATE_INFO:
      bytes_processed += PACKET_SIZE_ROBOT_STATE_INFO;
      success = handleRobotStateInfo(packet);
      break;

    default:
      sprintf(logBuffer, "[handlePacket] Error! At %d of %d bytes. [@] = %d\n", packet, packet_length, packet[bytes_processed]);
      return false;
    
    }
    
    if(!success) break;
  }

  if(!success)
    sprintf(logBuffer, "[handlePacket] Error! Could not parse packet with type %d\n", packet_type);

  return success;
}


/* Triggers when a call to HAL_SPI_TransmitReceive_DMA or HAL_SPI_TransmitReceive_IT (both non-blocking) completes */
void HAL_SPI_TxRxCpltCallback(SPI_HandleTypeDef *hspi){
  if(hspi->Instance == SX_TX->SPI->Instance){
    Wireless_DMA_Handler(SX_TX);
    // SX_TX should never receive a packet so that's why we don't call handlePacket here.
	}
	if(hspi->Instance == SX_RX->SPI->Instance) {
    Wireless_DMA_Handler(SX_RX);
    // First 3 bytes are status bytes
    handlePacket(SX_RX->RXbuf+3, SX_RX->payloadLength);
	}
}


/* External interrupt. Triggers when one of the SX1280 antennas has information available */
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin) {  
  // SX that sends packets wants to tell us something
  if (GPIO_Pin == SX_TX_IRQ.PIN) {
    Wireless_IRQ_Handler(SX_TX, 0, 0);
  }
  // SX that receives packets wants to tell us something
  if (GPIO_Pin == SX_RX_IRQ.PIN) {
    Wireless_IRQ_Handler(SX_RX, 0, 0);
    toggle_pin(LD_LED1);
  } 
}


/* Responsible for sending RobotCommand packages to the corresponding robots */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim){

  // TDMA Timer callback
  if(htim->Instance == htim1.Instance){
    static uint8_t idCounter = 0;

    // Keeps track of the total length of the packet that goes to the robot. 
    // Cannot exceed MAX_PACKET_SIZE, or it will overflow the internal buffer of the SX1280
    uint8_t total_packet_length = 0;    

    /* Add robot command */
    if(buffer_RobotCommand[idCounter].isNewPacket 
    && total_packet_length + PACKET_SIZE_ROBOT_COMMAND < MAX_PACKET_SIZE){
      buffer_RobotCommand[idCounter].isNewPacket = false;
      memcpy(sendBuffer + total_packet_length, buffer_RobotCommand[idCounter].packet.payload, PACKET_SIZE_ROBOT_COMMAND);
      total_packet_length += PACKET_SIZE_ROBOT_COMMAND;
    }

    /* Add buzzer command */
    if(buffer_RobotBuzzer[idCounter].isNewPacket
    && total_packet_length + PACKET_SIZE_ROBOT_BUZZER < MAX_PACKET_SIZE){
      buffer_RobotBuzzer[idCounter].isNewPacket = false;
      memcpy(sendBuffer + total_packet_length, buffer_RobotBuzzer[idCounter].packet.payload, PACKET_SIZE_ROBOT_BUZZER);
      total_packet_length += PACKET_SIZE_ROBOT_BUZZER;
    }
    
    /* Send new command if available for this robot ID */
    if(0 < total_packet_length){
      if(!isTransmitting){
        isTransmitting = true;
        SX_TX->SX_settings->syncWords[0] = robot_syncWord[idCounter];
        setSyncWords(SX_TX, SX_TX->SX_settings->syncWords[0], 0, 0);
        SendPacket(SX_TX, sendBuffer, total_packet_length);
      }
    }

    // Schedule next ID to be sent
    idCounter++;
    // Wrap around if the last ID has been dealt with
    if(MAX_ROBOT_ID < idCounter){
      idCounter = 0;
    }
  }
}