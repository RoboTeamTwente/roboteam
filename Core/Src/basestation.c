#include "basestation.h"
#include "main.h"
#include "Wireless.h"
#include "logging.h"
#include "packet_buffers.h"
#include "FT812Q_Drawing.h"

#include "REM_BaseTypes.h"
#include "REM_BasestationConfiguration.h"
#include "REM_BasestationSetConfiguration.h"
#include "REM_RobotCommand.h"
#include "REM_RobotFeedback.h"
#include "REM_RobotStateInfo.h"
#include "REM_RobotSetPIDGains.h"
#include "REM_SX1280Filler.h"

/* Counters, tracking the number of packets handled */ 
volatile int handled_RobotCommand = 0;
volatile int handled_RobotFeedback = 0;
volatile int handled_RobotBuzzer = 0;
volatile int handled_RobotStateInfo = 0;
volatile int handled_RobotGetPIDGains = 0;
volatile int handled_RobotSetPIDGains = 0;
volatile int handled_RobotPIDGains = 0;


/* Import hardware handles from main.c */
extern SPI_HandleTypeDef hspi1;
extern SPI_HandleTypeDef hspi2;
extern TIM_HandleTypeDef htim1;

/* Screen variables */
DISPLAY_STATES displayState = DISPLAY_STATE_DEINITIALIZED;
uint16_t touchPoint[2] = {-1, -1}; // Initialize touchPoint outside of screen, meaning TOUCH_STATE_RELEASED
TouchState touchState; // TODO check default initialization. What is touchState->state? Compiler dependent?

// Based on Wireless.c:SX1280_Settings.TXoffset
// Currently, we're splitting the SX1280 256 byte buffer in half. 128 for sending, 128 for receiving
// Set to 127, because that's the max value as defined in the datasheet
// Table 14-38: Payload Length Definition in FLRC Packet, page 124
#define MAX_PACKET_SIZE 127

/* SX data */
// TODO: Maybe move all configs to its own file? (basestation_config.c/h???)
extern SX1280_Settings SX1280_DEFAULT_SETTINGS;
static Wireless SX1280_TX = {0};
static Wireless SX1280_RX = {0};
static Wireless* SX_TX = &SX1280_TX;
static Wireless* SX_RX = &SX1280_RX;
static uint8_t SXTX_TX_buffer[MAX_PAYLOAD_SIZE + 3] __attribute__((aligned(4))) = {0};
static uint8_t SXTX_RX_buffer[MAX_PAYLOAD_SIZE + 3] __attribute__((aligned(4))) = {0};
static uint8_t SXRX_TX_buffer[MAX_PAYLOAD_SIZE + 3] __attribute__((aligned(4))) = {0};
static uint8_t SXRX_RX_buffer[MAX_PAYLOAD_SIZE + 3] __attribute__((aligned(4))) = {0};

static Wireless_Packet txPacket;
static Wireless_Packet rxPacket;

// The pins cannot be set at this point as they are not "const" enough for the compiler, so set them in the init
SX1280_Interface SX_TX_Interface = {.SPI= &hspi1, .TXbuf= SXTX_TX_buffer, .RXbuf= SXTX_RX_buffer, .logger=LOG_printf,};
SX1280_Interface SX_RX_Interface = {.SPI= &hspi2, .TXbuf= SXRX_TX_buffer, .RXbuf= SXRX_RX_buffer, .logger=LOG_printf,};

void Wireless_Writepacket_Cplt(void){
  TransmitPacket(SX_TX);
}
void Wireless_Readpacket_Cplt(void){
  handlePacket(rxPacket.message, rxPacket.payloadLength);
};

void Wireless_TXDone(SX1280_Packet_Status *status){
  toggle_pin(LD_TX);
}

void Wireless_RXDone(SX1280_Packet_Status *status){
  toggle_pin(LD_RX);
  toggle_pin(LD_LED2);
  /* It is possible that random noise can trigger the syncword. 
    * Syncword is 32 bits. Noise comes in at 2.4GHz. Syncword resets when wrong bit is received.
    * Expected length of wrong syncword is 1*0.5 + 2*0.25 + 3*0.125 + ... = 2
    * 2^32 combinations / (2400000000 / 2) syncwords = correct syncword every 3.57 seconds purely from noise
  */
  // Correct syncword from noise have a very weak signal 
  // Threshold is at -160/2 = -80 dBm
  if (status->RSSISync < 160) {
    ReadPacket_DMA(SX_RX, &rxPacket, &Wireless_Readpacket_Cplt);
    // not necessary to force WaitForPacket() here when configured in Rx Continuous mode
  }
}

void Wireless_RXTXTimeout(void){
  // Did not receive packet from robot. Should never be triggered, since the receiving SX is in continuous receiving mode
  toggle_pin(LD_LED3);
}

Wireless_IRQcallbacks SXTX_IRQcallbacks = {.txdone= &Wireless_TXDone, .rxdone= NULL,              .rxtxtimeout= &Wireless_RXTXTimeout};
Wireless_IRQcallbacks SXRX_IRQcallbacks = {.txdone= NULL,             .rxdone= &Wireless_RXDone,  .rxtxtimeout= &Wireless_RXTXTimeout};



/* Flags */
volatile bool flagHandleConfiguration = false;

// screenCounter acts as a timer for updating the screen
uint32_t screenCounter = 0;

/* Tracks time since last heartbeat. Runs at 1Hz */
uint32_t heartbeat_1000ms = 0;



void init(){
    HAL_Delay(1000); // TODO Why do we have this again? To allow for USB to start up iirc?
    LOG_init();
    // Init SX_TX
    LOG("[init] Initializing SX_TX\n");
    Wireless_Error err;
    SX_TX_Interface.BusyPin = SX_TX_BUSY;
    SX_TX_Interface.CS= SX_TX_CS;
    SX_TX_Interface.Reset= SX_TX_RST;
    err = Wireless_setPrint_Callback(SX_TX, NULL);
    err = Wireless_Init(SX_TX, SX1280_DEFAULT_SETTINGS, &SX_TX_Interface);
    err = Wireless_setIRQ_Callbacks(SX_TX,&SXTX_IRQcallbacks);
    if(err != WIRELESS_OK){
      //TODO: What do?
      while(1);
    }
    Wireless_setChannel(SX_TX, YELLOW_CHANNEL);
    
    // Init SX_RX
    LOG("[init] Initializing SX_RX\n");
    SX_RX_Interface.BusyPin= SX_RX_BUSY;
    SX_RX_Interface.CS= SX_RX_CS;
    SX_RX_Interface.Reset= SX_RX_RST;
    err = Wireless_setPrint_Callback(SX_TX, NULL);
    err = Wireless_Init(SX_RX, SX1280_DEFAULT_SETTINGS, &SX_RX_Interface);
    err = Wireless_setIRQ_Callbacks(SX_RX,&SXRX_IRQcallbacks);
    if(err != WIRELESS_OK){
      //TODO: What do?
      while(1);
    }
    // TODO: Use proper defines
    Wireless_setChannel(SX_RX, YELLOW_CHANNEL);
    // Set SX_RX syncword to basestation syncword
    uint32_t syncwords[2] = {robot_syncWord[16],0};
    Wireless_setRXSyncwords(SX_RX, syncwords);

    // Start listening on the SX_RX for packets from the robots
    WaitForPacketContinuous(SX_RX);

    // Start the timer that is responsible for sending packets to the robots
    // With 16 robots at 60Hz each, this timer runs at approximately 960Hz
    LOG("[init] Initializing Timer\n");
    HAL_TIM_Base_Start_IT(&htim1);

    // display_Init();
    // displayState = DISPLAY_STATE_INITIALIZED;
    // drawBasestation(true);

    // Initialize the REM_SX1280FillerPayload packet
    REM_SX1280Filler filler;
    filler.header = PACKET_TYPE_REM_SX1280FILLER;
    filler.remVersion = LOCAL_REM_VERSION;
    encodeREM_SX1280Filler(&SX1280_filler_payload, &filler);

    LOG("[init] Initializion complete\n");
}



void loop(){

  LOG_send();

  /* Heartbeat every second */
  if(heartbeat_1000ms + 1000 < HAL_GetTick()){
    heartbeat_1000ms += 1000;
    // HexOut("Tick!\n", 6);
    LOG_printf("Tick | RC %d RF %d RB %d RSI %d GPID %d PID %d\n",
    handled_RobotCommand, handled_RobotFeedback, handled_RobotBuzzer, handled_RobotStateInfo, handled_RobotGetPIDGains, handled_RobotPIDGains);
    toggle_pin(LD_ACTIVE);
  }

  // TODO put multiple of these messages into a single USB packet, instead of sending every packet separately
  /* Send any new RobotFeedback packets */
  for(int id = 0; id <= MAX_ROBOT_ID; id++){
    if(buffer_RobotFeedback[id].isNewPacket){
      LOG_sendBlocking(buffer_RobotFeedback[id].packet.payload, PACKET_SIZE_REM_ROBOT_FEEDBACK);
      buffer_RobotFeedback[id].isNewPacket = false;
    }
  }

  /* Send any new RobotStateInfo packets */
  for(int id = 0; id <= MAX_ROBOT_ID; id++){
    if(buffer_RobotStateInfo[id].isNewPacket){
      LOG_sendBlocking(buffer_RobotStateInfo[id].packet.payload, PACKET_SIZE_REM_ROBOT_STATE_INFO);
      buffer_RobotStateInfo[id].isNewPacket = false;
    }
  }

  /* Send any new RobotPIDGains packets */
  for(int id = 0; id <= MAX_ROBOT_ID; id++){
    if(buffer_RobotPIDGains[id].isNewPacket){
      LOG_sendBlocking(buffer_RobotPIDGains[id].packet.payload, PACKET_SIZE_REM_ROBOT_PIDGAINS);
      buffer_RobotPIDGains[id].isNewPacket = false;
    }
  }

  if (flagHandleConfiguration) {
    // TODO: Make a nice function for this
    REM_BasestationConfiguration configuration;
    configuration.header = PACKET_TYPE_REM_BASESTATION_CONFIGURATION;
    configuration.remVersion = LOCAL_REM_VERSION;
    configuration.channel = Wireless_getChannel(SX_TX);

    REM_BasestationConfigurationPayload payload;
    encodeREM_BasestationConfiguration(&payload, &configuration);

    LOG_sendBlocking(payload.payload, PACKET_SIZE_REM_BASESTATION_CONFIGURATION);
    flagHandleConfiguration = false;
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
 * If so, it moves the packet to the RobotCommand buffer, and sets a flag to send the packet to
 * the corresponding robot.
 * 
 * @param packet_buffer Pointer to the buffer that holds the packet
 */
void handleRobotCommand(uint8_t* packet_buffer){
  handled_RobotCommand++;

  // Check if the packet REM version corresponds to the local REM version. If the REM versions do not correspond, drop the packet.
  uint8_t packet_rem_version = REM_RobotCommand_get_remVersion((REM_RobotCommandPayload*) packet_buffer);
  if(packet_rem_version != LOCAL_REM_VERSION){
    LOG_printf("[handleRobotCommand] Error! packet_rem_version %u != %u LOCAL_REM_VERSION.", packet_rem_version, LOCAL_REM_VERSION);
    return;
  }

  // Store the message in the RobotCommand buffer. Set flag indicating packet needs to be sent to the robot
  uint8_t robot_id = REM_RobotCommand_get_id((REM_RobotCommandPayload*) packet_buffer);
  memcpy(buffer_RobotCommand[robot_id].packet.payload, packet_buffer, PACKET_SIZE_REM_ROBOT_COMMAND);
  buffer_RobotCommand[robot_id].isNewPacket = true;
  buffer_RobotCommand[robot_id].counter++;
}

void handleRobotSetPIDGains(uint8_t* packet_buffer){
  handled_RobotSetPIDGains++;

  // Check if the packet REM version corresponds to the local REM version. If the REM versions do not correspond, drop the packet.
  uint8_t packet_rem_version = REM_RobotSetPIDGains_get_remVersion((REM_RobotSetPIDGainsPayload*) packet_buffer);
  if(packet_rem_version != LOCAL_REM_VERSION){
    LOG_printf("[handleRobotSetPIDGains] Error! packet_rem_version %u != %u LOCAL_REM_VERSION.", packet_rem_version, LOCAL_REM_VERSION);
    return;
  }

  // Store the message in the RobotSetPIDGains buffer. Set flag indicating packet needs to be sent to the robot
  uint8_t robot_id = REM_RobotSetPIDGains_get_id((REM_RobotSetPIDGainsPayload*) packet_buffer);
  memcpy(buffer_RobotSetPIDGains[robot_id].packet.payload, packet_buffer, PACKET_SIZE_REM_ROBOT_SET_PIDGAINS);
  buffer_RobotSetPIDGains[robot_id].isNewPacket = true;
  buffer_RobotSetPIDGains[robot_id].counter++;
}


/**
 * @brief Receives a buffer which is assumed to be holding a RobotFeedback packet. 
 * If so, it moves the packet to the RobotFeedback buffer, and sets a flag to send the packet to
 * the computer.
 * 
 * @param packet_buffer Pointer to the buffer that holds the packet
 */
void handleRobotFeedback(uint8_t* packet_buffer){
  handled_RobotFeedback++;
  
  // Check if the packet REM version corresponds to the local REM version. If the REM versions do not correspond, drop the packet.
  uint8_t packet_rem_version = REM_RobotFeedback_get_remVersion((REM_RobotFeedbackPayload*) packet_buffer);
  if(packet_rem_version != LOCAL_REM_VERSION){
    LOG_printf("[handleRobotFeedback] Error! packet_rem_version %u != %u LOCAL_REM_VERSION.", packet_rem_version, LOCAL_REM_VERSION);
    return;
  }

  // Store the message in the RobotFeedback buffer. Set flag indicating packet needs to be sent to the robot
  uint8_t robot_id = REM_RobotFeedback_get_id((REM_RobotFeedbackPayload*) packet_buffer);
  memcpy(buffer_RobotFeedback[robot_id].packet.payload, packet_buffer, PACKET_SIZE_REM_ROBOT_FEEDBACK);
  buffer_RobotFeedback[robot_id].isNewPacket = true;
  buffer_RobotFeedback[robot_id].counter++;
}


/**
 * @brief Receives a buffer which is assumed to be holding a RobotStateInfo packet. 
 * If so, it moves the packet to the RobotStateInfo buffer, and sets a flag to send the packet to
 * the computer.
 * 
 * @param packet_buffer Pointer to the buffer that holds the packet
 */
void handleRobotStateInfo(uint8_t* packet_buffer){
  handled_RobotStateInfo++;
  
  // Check if the packet REM version corresponds to the local REM version. If the REM versions do not correspond, drop the packet.
  uint8_t packet_rem_version = REM_RobotStateInfo_get_remVersion((REM_RobotStateInfoPayload*) packet_buffer);
  if(packet_rem_version != LOCAL_REM_VERSION){
    LOG_printf("[handleRobotStateInfo] Error! packet_rem_version %u != %u LOCAL_REM_VERSION.", packet_rem_version, LOCAL_REM_VERSION);
    return;
  }

  // Store the message in the RobotStateInfo buffer. Set flag to be sent to the robot
  uint8_t robot_id = REM_RobotStateInfo_get_id((REM_RobotStateInfoPayload*) packet_buffer);
  memcpy(buffer_RobotStateInfo[robot_id].packet.payload, packet_buffer, PACKET_SIZE_REM_ROBOT_STATE_INFO);
  buffer_RobotStateInfo[robot_id].isNewPacket = true;
  buffer_RobotStateInfo[robot_id].counter++;
}


/**
 * @brief Receives a buffer which is assumed to be holding a RobotBuzzer packet. 
 * If so, it moves the packet to the RobotBuzzer buffer, and sets a flag to send the packet to
 * the corresponding robot.
 * 
 * @param packet_buffer Pointer to the buffer that holds the packet
 */
void handleRobotBuzzer(uint8_t* packet_buffer){
  handled_RobotBuzzer++;
  
  // Check if the packet REM version corresponds to the local REM version. If the REM versions do not correspond, drop the packet.
  uint8_t packet_rem_version = REM_RobotBuzzer_get_remVersion((REM_RobotBuzzerPayload*) packet_buffer);
  if(packet_rem_version != LOCAL_REM_VERSION){
    LOG_printf("[handleRobotBuzzer] Error! packet_rem_version %u != %u LOCAL_REM_VERSION.", packet_rem_version, LOCAL_REM_VERSION);
    return;
  }

  // Store the message in the RobotBuzzer buffer. Set flag to be sent to the robot
  uint8_t robot_id = REM_RobotBuzzer_get_id((REM_RobotBuzzerPayload*) packet_buffer);
  memcpy(buffer_RobotBuzzer[robot_id].packet.payload, packet_buffer, PACKET_SIZE_REM_ROBOT_BUZZER);
  buffer_RobotBuzzer[robot_id].isNewPacket = true;
  buffer_RobotBuzzer[robot_id].counter++;
}

void handleBasestationSetConfiguration(uint8_t* packet_buffer){
  // Check if the packet REM version corresponds to the local REM version. If the REM versions do not correspond, drop the packet.
  uint8_t packet_rem_version = REM_BasestationSetConfiguration_get_remVersion((REM_BasestationSetConfigurationPayload*) packet_buffer);
  //uint8_t packet_rem_version = RobotBuzzer_get_remVersion((RobotBuzzerPayload*) packet_buffer);
  if(packet_rem_version != LOCAL_REM_VERSION){
    LOG_printf("[handleBasestationSetConfiguration] Error! packet_rem_version %u != %u LOCAL_REM_VERSION.", packet_rem_version, LOCAL_REM_VERSION);
    return;
  }

  WIRELESS_CHANNEL newChannel = REM_BasestationSetConfiguration_get_channel((REM_BasestationSetConfigurationPayload*) packet_buffer);
  Wireless_setChannel(SX_TX, newChannel);
  Wireless_setChannel(SX_RX, newChannel);
}

void handleRobotGetPIDGains(uint8_t* packet_buffer){
  handled_RobotGetPIDGains++;
  
  // Check if the packet REM version corresponds to the local REM version. If the REM versions do not correspond, drop the packet.
  uint8_t packet_rem_version = REM_RobotGetPIDGains_get_remVersion((REM_RobotGetPIDGainsPayload*) packet_buffer);
  if(packet_rem_version != LOCAL_REM_VERSION){
    LOG_printf("[handleRobotGetPIDGains] Error! packet_rem_version %u != %u LOCAL_REM_VERSION.", packet_rem_version, LOCAL_REM_VERSION);
    return;
  }

  // Store the message in the RobotGetPIDGains buffer. Set flag to be sent to the robot
  uint8_t robot_id = REM_RobotGetPIDGains_get_id((REM_RobotGetPIDGainsPayload*) packet_buffer);
  memcpy(buffer_RobotGetPIDGains[robot_id].packet.payload, packet_buffer, PACKET_SIZE_REM_ROBOT_GET_PIDGAINS);
  buffer_RobotGetPIDGains[robot_id].isNewPacket = true;
  buffer_RobotGetPIDGains[robot_id].counter++;
}

void handleRobotPIDGains(uint8_t* packet_buffer){
  handled_RobotPIDGains++;
  
  // Check if the packet REM version corresponds to the local REM version. If the REM versions do not correspond, drop the packet.
  uint8_t packet_rem_version = REM_RobotPIDGains_get_remVersion((REM_RobotPIDGainsPayload*) packet_buffer);
  if(packet_rem_version != LOCAL_REM_VERSION){
    LOG_printf("[handleRobotPIDGains] Error! packet_rem_version %u != %u LOCAL_REM_VERSION.", packet_rem_version, LOCAL_REM_VERSION);
    return;
  }

  // Store the message in the RobotGetPIDGains buffer. Set flag to be sent to the robot
  uint8_t robot_id = REM_RobotPIDGains_get_id((REM_RobotPIDGainsPayload*) packet_buffer);
  memcpy(buffer_RobotPIDGains[robot_id].packet.payload, packet_buffer, PACKET_SIZE_REM_ROBOT_PIDGAINS);
  buffer_RobotPIDGains[robot_id].isNewPacket = true;
  buffer_RobotPIDGains[robot_id].counter++;
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
 * @param packet_buffer Pointer to the buffer the packet from the USB is stored in
 * @param packet_length Length of the packet currently in the buffer
 * @return true if the packet(s) have been handled succesfully
 * @return false if the packet(s) have been handled unsuccessfully, e.g. due to corruption
 */
bool handlePacket(uint8_t* packet_buffer, uint32_t packet_length){
  uint8_t packet_type;
  uint32_t bytes_processed = 0;

  while(bytes_processed < packet_length){

    packet_type = packet_buffer[bytes_processed];

    switch (packet_type){

      case PACKET_TYPE_REM_ROBOT_COMMAND:
        handleRobotCommand(packet_buffer + bytes_processed);
        bytes_processed += PACKET_SIZE_REM_ROBOT_COMMAND;
        break;
      
      case PACKET_TYPE_REM_ROBOT_SET_PIDGAINS:
        handleRobotSetPIDGains(packet_buffer + bytes_processed);
        bytes_processed += PACKET_SIZE_REM_ROBOT_SET_PIDGAINS;
        break;
      
      case PACKET_TYPE_REM_ROBOT_FEEDBACK:
        handleRobotFeedback(packet_buffer + bytes_processed);
        bytes_processed += PACKET_SIZE_REM_ROBOT_FEEDBACK;
        break;
      
      case PACKET_TYPE_REM_ROBOT_BUZZER:
        handleRobotBuzzer(packet_buffer + bytes_processed);
        bytes_processed += PACKET_SIZE_REM_ROBOT_BUZZER;
        break;

      case PACKET_TYPE_REM_ROBOT_STATE_INFO:
        handleRobotStateInfo(packet_buffer + bytes_processed);
        bytes_processed += PACKET_SIZE_REM_ROBOT_STATE_INFO;
        break;
      
      case PACKET_TYPE_REM_BASESTATION_SET_CONFIGURATION:
        handleBasestationSetConfiguration(packet_buffer + bytes_processed);
        bytes_processed += PACKET_SIZE_REM_BASESTATION_SET_CONFIGURATION;
        break;

      case PACKET_TYPE_REM_ROBOT_GET_PIDGAINS:
        handleRobotGetPIDGains(packet_buffer + bytes_processed);
        bytes_processed += PACKET_TYPE_REM_ROBOT_GET_PIDGAINS;
        break;

      case PACKET_TYPE_REM_ROBOT_PIDGAINS:
        handleRobotPIDGains(packet_buffer + bytes_processed);
        bytes_processed += PACKET_TYPE_REM_ROBOT_PIDGAINS;
        break;

      case PACKET_TYPE_REM_BASESTATION_GET_CONFIGURATION:
        bytes_processed += PACKET_SIZE_REM_BASESTATION_GET_CONFIGURATION;
        flagHandleConfiguration = true;
        break;

      default:
        LOG_printf("[handlePacket] Error! At %ld of %ld bytes. [@] = %d\n", bytes_processed, packet_length, packet_buffer[bytes_processed]);
        return false;
    }
  }

  return true;
}



/* Triggers when a call to HAL_SPI_TransmitReceive_DMA or HAL_SPI_TransmitReceive_IT (both non-blocking) completes */
void HAL_SPI_TxRxCpltCallback(SPI_HandleTypeDef *hspi){
  if(hspi->Instance == SX_TX->Interface->SPI->Instance){
    Wireless_DMA_Handler(SX_TX);
    // SX_TX should never receive a packet so that's why we don't call handlePacket here.
	}
	if(hspi->Instance == SX_RX->Interface->SPI->Instance) {
    Wireless_DMA_Handler(SX_RX);
	}
}


/* External interrupt. Triggers when one of the SX1280 antennas has information available */
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin) {  
  // SX that sends packets wants to tell us something
  if (GPIO_Pin == SX_TX_IRQ.PIN) {
    Wireless_IRQ_Handler(SX_TX);
  }
  // SX that receives packets wants to tell us something
  if (GPIO_Pin == SX_RX_IRQ.PIN) {
    Wireless_IRQ_Handler(SX_RX);
    toggle_pin(LD_LED1);
  } 
}


/* Responsible for sending RobotCommand packets to the corresponding robots */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim){

  // TDMA Timer callback, runs at approximately 960Hz
  /* Every millisecond, a transmission can be made to a single robot. As per the TDMA protocol, the robot has
  * to respond with any of its own packets within this millisecond. The code loops through all possible robot 
  * id's (currently 0 to 15), incrementing the id it sends packets to, every millisecond. A transmission to 
  * the robot can consist of multiple packet, for example a RobotCommand and a RobotBuzzer. These packets 
  * are stitched together and sent to the robot in a single transmission. A single transmission can never 
  * exceed more than 127 bytes, per the SX1280 datasheet and our settings. */

  if(htim->Instance == htim1.Instance){
    // Counter that tracks the current robot id that the basestation sends a packet to
    static uint8_t idCounter = 0;

    // Keeps track of the total length of the packet that goes to the robot. 
    // Cannot exceed MAX_PACKET_SIZE, or it will overflow the internal buffer of the SX1280
    uint8_t total_packet_length = 0;    

    /* Add RobotCommand to the transmission */
    if(buffer_RobotCommand[idCounter].isNewPacket 
    && total_packet_length + PACKET_SIZE_REM_ROBOT_COMMAND < MAX_PACKET_SIZE){
      buffer_RobotCommand[idCounter].isNewPacket = false;
      memcpy(txPacket.message + total_packet_length, buffer_RobotCommand[idCounter].packet.payload, PACKET_SIZE_REM_ROBOT_COMMAND);
      total_packet_length += PACKET_SIZE_REM_ROBOT_COMMAND;
    }

    /* Add RobotSetPIDGains to the transmission */
    if(buffer_RobotSetPIDGains[idCounter].isNewPacket
    && total_packet_length + PACKET_SIZE_REM_ROBOT_SET_PIDGAINS < MAX_PACKET_SIZE){
      buffer_RobotSetPIDGains[idCounter].isNewPacket = false;
      memcpy(txPacket.message + total_packet_length, buffer_RobotSetPIDGains[idCounter].packet.payload, PACKET_SIZE_REM_ROBOT_SET_PIDGAINS);
      total_packet_length += PACKET_SIZE_REM_ROBOT_SET_PIDGAINS;
    }

    /* Add RobotBuzzer to the transmission */
    if(buffer_RobotBuzzer[idCounter].isNewPacket
    && total_packet_length + PACKET_SIZE_REM_ROBOT_BUZZER < MAX_PACKET_SIZE){
      buffer_RobotBuzzer[idCounter].isNewPacket = false;
      memcpy(txPacket.message + total_packet_length, buffer_RobotBuzzer[idCounter].packet.payload, PACKET_SIZE_REM_ROBOT_BUZZER);
      total_packet_length += PACKET_SIZE_REM_ROBOT_BUZZER;
    }
    
    /* Add RobotGetPIDGains to the transmission */
    if(buffer_RobotGetPIDGains[idCounter].isNewPacket && total_packet_length + PACKET_SIZE_REM_ROBOT_GET_PIDGAINS < MAX_PACKET_SIZE){
      buffer_RobotGetPIDGains[idCounter].isNewPacket = false;
      memcpy(txPacket.message + total_packet_length, buffer_RobotGetPIDGains[idCounter].packet.payload, PACKET_SIZE_REM_ROBOT_GET_PIDGAINS);
      total_packet_length += PACKET_SIZE_REM_ROBOT_GET_PIDGAINS;
    }

    /* Send new command if available for this robot ID */
    if(0 < total_packet_length){
      if(SX_TX->state == WIRELESS_READY){
        /* Add a filler packet to the buffer if there are currently less than 6 bytes in the buffer
        * The minimum payload size for the SX1280 in FLRC mode is 6 bytes. 
        * See documentation page 124 - Table 14-36: Sync Word Combination in FLRC Packet */
        if(total_packet_length < 6){
          memcpy(txPacket.message + total_packet_length, SX1280_filler_payload.payload, PACKET_SIZE_REM_SX1280FILLER);
          total_packet_length += PACKET_SIZE_REM_SX1280FILLER;
        }

        txPacket.payloadLength = total_packet_length;
        Wireless_setTXSyncword(SX_TX,robot_syncWord[idCounter]);
        WritePacket_DMA(SX_TX, &txPacket, &Wireless_Writepacket_Cplt);
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