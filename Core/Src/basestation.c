#include "basestation.h"
#include "main.h"
#include "Wireless.h"
#include "TextOut.h"
#include "msg_buff.h"
#include "FT812Q_Drawing.h"

#include "BaseTypes.h"
#include "BasestationStatistics.h"
#include "RobotCommand.h"
#include "CircularBuffer.h"

volatile int I1 = 0;
volatile int cFeedback = 0;
volatile int Iusb = 0;

extern SPI_HandleTypeDef hspi1;
extern SPI_HandleTypeDef hspi2;
extern TIM_HandleTypeDef htim1;

extern uint32_t usbLength;
extern uint8_t usbData[64];

/* Screen variables */
DISPLAY_STATES displayState = DISPLAY_STATE_DEINITIALIZED;
uint16_t touchPoint[2] = {-1, -1}; // Initialize touchPoint outside of screen, meaning TOUCH_STATE_RELEASED
TouchState touchState; // TODO check default initialization. What is touchState->state? Compiler dependent?
char logBuffer[100];

/* Flags */
volatile bool flagHandleStatistics = false;

void init(){
    HAL_Delay(1000); // TODO Why do we have this again? To allow for USB to start up iirc?
    
    LOG("[init] Initializing SX_TX\n");
    SX_TX = Wireless_Init(COMMAND_CHANNEL, &hspi1, 0);
    
    LOG("[init] Initializing SX_RX\n");
    SX_RX = Wireless_Init(FEEDBACK_CHANNEL, &hspi2, 1);
    SX_RX->SX_settings->syncWords[0] = robot_syncWord[16];
    setSyncWords(SX_RX, SX_RX->SX_settings->syncWords[0], 0x00, 0x00);
    setRX(SX_RX, SX_RX->SX_settings->periodBase, 0xFFFF);

    LOG("[init] Initializing Timer\n");
    HAL_TIM_Base_Start_IT(&htim1);

    // display_Init();
    // displayState = DISPLAY_STATE_INITIALIZED;
    // drawBasestation(true);

    LOG("[init] Initializion complete\n");
}

// screenCounter acts as a timer for updating the screen
uint32_t screenCounter = 0;

void loop(){
  
  /* Send any new feedback packets */
  for(int id = 0; id < 16; id++){
    if(msgBuff[id].isNewFeedback){
      HexOut(msgBuff[id].feedback.payload, PACKET_SIZE_ROBOT_FEEDBACK);
      msgBuff[id].isNewFeedback = false;
      msgBuff[id].packetsReceived++;
    }
  }

  if(flagHandleStatistics){
    handleStatistics();
    flagHandleStatistics = false;
  }

  /* Ensures that the CPU doesn't get overloaded with display stuff. Make better solution for this? */
  if(screenCounter++ < 80000)
    return;
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
 * multiple touch events as a single screen press, keeps track on
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
 * @param Len Length of the packet
 * @return true if the packet has been handled succesfully
 * @return false if there was something wrong with the packet
 */
bool handleRobotCommand(uint8_t* Buf, uint32_t Len){
  // Check if the packet has the correct size
  if(Len != PACKET_SIZE_ROBOT_COMMAND)
    return false;

  // Interpret buffer as a RobotCommandPayload
  RobotCommandPayload *rc = (RobotCommandPayload*) Buf;

  // Check if the robotId is valid
  uint8_t robotId = RobotCommand_getId(rc);
  if (16 <= robotId)
    return false;
    
  // Store the message in the buffer. Set flag to be sent to the robot
  memcpy(msgBuff[robotId].command.payload, Buf, PACKET_SIZE_ROBOT_COMMAND);
  msgBuff[robotId].isNewCommand = true;
  
  return true;
}



/**
 * @brief Handles sending basestation statistics over USB.
 * 
 * @return true if the packet has been handled succesfully
 * @return false if there was something wrong with the packet
 */
bool handleStatistics(void){
  basestationStatistics bs = {0};
  bs.header = PACKET_TYPE_BASESTATION_STATISTICS;
  for(int ID = 0; ID < 16; ID++){
    // Send ratio + packets sent (clipped) for each robot
    // TODO this clipping is not nice at all. Next to that, we need to find some way to figure out
    // how to draw if robots are "active", meaning we need to reset these stats every few ms or so.
    bs.robot[ID].packetsReceived  = msgBuff[ID].packetsReceived >= 255 ? 255 : msgBuff[ID].packetsReceived;
    bs.robot[ID].packetsSent      = msgBuff[ID].packetsSent     >= 255 ? 255 : msgBuff[ID].packetsSent;
    // Reset statistics
    msgBuff[ID].packetsSent = 0;
    msgBuff[ID].packetsReceived = 0;
  }
  HexOut(bs.payload, PACKET_SIZE_BASESTATION_STATISTICS);
  return true;
}



/**
 * @brief routes any incoming packet to the correct function. Hub for all incoming packets.
 * TODO actually make it route all incoming packets, and not just USB packets
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
bool handlePacket(uint8_t* Buf, uint32_t *Len){

  uint8_t packetType = Buf[0];
  uint32_t packetSize = *Len;
  
  Iusb++;

  bool success = false;

  switch(packetType){
    case PACKET_TYPE_ROBOT_COMMAND:
      success = handleRobotCommand(Buf, packetSize);
      break;
    case PACKET_TYPE_BASESTATION_GET_STATISTICS:
      flagHandleStatistics = true;
      break;
    default:
      break;
  }

  return success;
}

/* Triggers when a call to HAL_SPI_TransmitReceive_DMA or HAL_SPI_TransmitReceive_IT (both non-blocking) completes */
void HAL_SPI_TxRxCpltCallback(SPI_HandleTypeDef *hspi){
    if(hspi->Instance == SX_TX->SPI->Instance) {
		I1++;
    Wireless_DMA_Handler(SX_TX);
	}
	if(hspi->Instance == SX_RX->SPI->Instance) {
    Wireless_DMA_Handler(SX_RX);
	}
}

/* External interrupt. Triggers when one of the SX1280 antennas has information available */
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin) {
    
	if (GPIO_Pin == SX_TX_IRQ.PIN) {
		Wireless_IRQ_Handler(SX_TX, 0, 0);
        I1++;
	}
	if (GPIO_Pin == SX_RX_IRQ.PIN) {
		Wireless_IRQ_Handler(SX_RX, 0, 0);
        cFeedback++;
		toggle_pin(LD_LED1);
	}
    
}

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
  // TDMA Timer callback
  if(htim->Instance == htim1.Instance){
    static uint8_t idCounter = 0;
    /* Send new command if available for this robot ID */
    if(msgBuff[idCounter].isNewCommand){
      if(!isTransmitting){
        isTransmitting = true;
        msgBuff[idCounter].packetsSent++;
        SX_TX->SX_settings->syncWords[0] = robot_syncWord[idCounter];
        setSyncWords(SX_TX, SX_TX->SX_settings->syncWords[0], 0, 0);
        SendPacket(SX_TX, msgBuff[idCounter].command.payload+2, PACKET_SIZE_ROBOT_COMMAND-2);
        msgBuff[idCounter].isNewCommand = false;
      }
    }
    // Schedule next ID to be sent
    idCounter++;
    // Wrap around if the last ID has been dealt with
    if(idCounter >= 16){
      idCounter = 0;
    }
  }
}