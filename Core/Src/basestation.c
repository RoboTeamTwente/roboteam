#include "basestation.h"
#include "main.h"
#include "Wireless.h"
#include "logging.h"
#include "packet_buffers.h"
#include "FT812Q_Drawing.h"

#include "REM_BaseTypes.h"
#include "REM_BasestationConfiguration.h"
#include "REM_RobotCommand.h"
#include "REM_RobotFeedback.h"
#include "REM_RobotStateInfo.h"
#include "REM_RobotSetPIDGains.h"
#include "REM_RobotMusicCommand.h"
#include "REM_Packet.h"
#include "REM_SX1280Filler.h"

#include "CircularBuffer.h"

//TODO REMOVE
#include "usbd_cdc.h"
#include "usb_device.h"
#include "usbd_cdc_if.h"
extern USBD_HandleTypeDef hUsbDeviceFS;



/* Counters, tracking the number of packets handled */ 
volatile int handled_RobotCommand = 0;
volatile int handled_RobotFeedback = 0;
volatile int handled_RobotBuzzer = 0;
volatile int handled_RobotStateInfo = 0;
volatile int handled_RobotGetPIDGains = 0;
volatile int handled_RobotSetPIDGains = 0;
volatile int handled_RobotPIDGains = 0;
volatile int handled_RobotMusicCommand = 0;
volatile int handled_INVALID = 0;
volatile int handled_INVALID_nbytes = 0;
volatile char INVALID_buffer[1024];

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
SX1280_Interface SX_TX_Interface = {.SPI= &hspi1, .TXbuf= SXTX_TX_buffer, .RXbuf= SXTX_RX_buffer, /*.logger=LOG_printf*/};
SX1280_Interface SX_RX_Interface = {.SPI= &hspi2, .TXbuf= SXRX_TX_buffer, .RXbuf= SXRX_RX_buffer, /*.logger=LOG_printf*/};

void Wireless_Writepacket_Cplt(void){
  TransmitPacket(SX_TX);
}
void Wireless_Readpacket_Cplt(void){
  handlePackets(rxPacket.message, rxPacket.payloadLength);
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


uint8_t stringbuffer[1024];
extern UART_HandleTypeDef huart3;

void init(){
    HAL_Delay(1000); // TODO Why do we have this again? To allow for USB to start up iirc?
    

    LOG_init();
    static char charbuffer[100];
    sprintf(charbuffer, "Yellow World!\n");

    // while(1){
    //   LOG("Hello world\n");
    //   LOG_sendBlocking(charbuffer, strlen(charbuffer));
    //   // CDC_Transmit_FS(charbuffer, strlen(charbuffer));
    //   // toggle_pin(LD_ACTIVE);
    //   // HAL_UART_Transmit(&huart3, charbuffer, strlen(charbuffer), 10);
    //   HAL_GPIO_TogglePin(LD_LED2_GPIO_Port, LD_LED2_Pin);
    //   HAL_Delay(1000);
    // }

    // Init SX_TX
    /*
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
    */

    // Init SX_RX
    // LOG("[init] Initializing SX_RX\n");
    /*
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
    // LOG("[init] Initializing Timer\n");
    HAL_TIM_Base_Start_IT(&htim1);
    */

    // display_Init();
    // displayState = DISPLAY_STATE_INITIALIZED;
    // drawBasestation(true);

    // Initialize the REM_SX1280FillerPayload packet
    REM_SX1280Filler filler;
    filler.header = REM_PACKET_TYPE_REM_SX1280FILLER;
    filler.remVersion = REM_LOCAL_VERSION;
    encodeREM_SX1280Filler(&SX1280_filler_payload, &filler);



    // Initialize the circular buffers for the nonprioriry queue
    // STM32F767 has 512kb of RAM. Give each robot a 10kb buffer for a total of 160kb RAM. 
    // That should be enough for around 200 messages per robot at least

    // for(uint8_t robot_id = 0; robot_id < MAX_NUMBER_OF_ROBOTS; robot_id++){
    //   nonpriority_queue_robots[robot_id] = CircularBuffer_init(false, 10000);
    // }
    // nonpriority_queue_pc = CircularBuffer_init(false, 10000);
    // nonpriority_queue_bs = CircularBuffer_init(false, 10000);

    for(uint8_t robot_id = 0; robot_id < MAX_NUMBER_OF_ROBOTS; robot_id++){
      nonpriority_queue_robots_index[robot_id] = CircularBuffer_init(true, 40);
    }
    nonpriority_queue_pc_index = CircularBuffer_init(true, 40);
    nonpriority_queue_bs_index = CircularBuffer_init(true, 40);
    //TODO: How to assign associated buffer?

    // LOG("[init] Initializion complete\n");
    // LOG_sendAll();
    // while(1);
}



void loop(){

  LOG_send();

  uint32_t current_time = HAL_GetTick();

  /* Heartbeat every second */
  if(heartbeat_1000ms + 1000 < current_time){
    HAL_GPIO_TogglePin(LD_LED2_GPIO_Port, LD_LED2_Pin);
    while (heartbeat_1000ms + 1000 < current_time)
      heartbeat_1000ms += 1000;

      USBD_CDC_HandleTypeDef* hcdc = (USBD_CDC_HandleTypeDef*)hUsbDeviceFS.pClassData;


    // LOG_printf("Tick | RC %d RF %d RB %d RSI %d GPID %d PID %d INV %d B %d\n",
    // handled_RobotCommand, handled_RobotFeedback, handled_RobotBuzzer, handled_RobotStateInfo, handled_RobotGetPIDGains, handled_RobotPIDGains, handled_INVALID, handled_INVALID_nbytes);
    // LOG_printf("\nTick | t=%d, INVALID %d INVALID_NBYTES %d\n", heartbeat_1000ms, handled_INVALID, handled_INVALID_nbytes);
    
    // sprintf(stringbuffer, "BTick | t=%d, INVALID %d INVALID_NBYTES %d\n", heartbeat_1000ms, handled_INVALID, handled_INVALID_nbytes);
    for(uint8_t wer = 0; wer < 10; wer++){
      sprintf(stringbuffer, " Hello%d %d\n", wer, current_time);
      while (hcdc->TxState != 0);
      CDC_Transmit_FS(stringbuffer, strlen(stringbuffer));
      HAL_Delay(2);
    }
    

    INVALID_buffer[handled_INVALID_nbytes+1] = "\n";
    INVALID_buffer[handled_INVALID_nbytes+2] = "\0";
    // LOG_sendBlocking(INVALID_buffer, handled_INVALID_nbytes+2);
    // HAL_UART_Transmit(&huart3, INVALID_buffer, handled_INVALID_nbytes+2, 10);


    // while (hcdc->TxState != 0);
    // CDC_Transmit_FS(INVALID_buffer, handled_INVALID_nbytes+2);




    handled_INVALID = 0;
    handled_INVALID_nbytes = 0;
    memset(INVALID_buffer, 0, 200);
    toggle_pin(LD_ACTIVE);
  }

  // TODO put multiple of these messages into a single USB packet, instead of sending every packet separately
  /* Send any new RobotFeedback packets */
  for(int id = 0; id <= MAX_ROBOT_ID; id++){
    if(buffer_REM_RobotFeedback[id].isNewPacket){
      //LOG_sendBlocking(buffer_REM_RobotFeedback[id].packet.payload, REM_PACKET_SIZE_REM_ROBOT_FEEDBACK);
      buffer_REM_RobotFeedback[id].isNewPacket = false;
    }
  }

  // /* Send any new RobotStateInfo packets */
  // for(int id = 0; id <= MAX_ROBOT_ID; id++){
  //   if(buffer_RobotStateInfo[id].isNewPacket){
  //     //LOG_sendBlocking(buffer_RobotStateInfo[id].packet.payload, REM_PACKET_SIZE_REM_ROBOT_STATE_INFO);
  //     buffer_RobotStateInfo[id].isNewPacket = false;
  //   }
  // }

  // /* Send any new RobotPIDGains packets */
  // for(int id = 0; id <= MAX_ROBOT_ID; id++){
  //   if(buffer_RobotPIDGains[id].isNewPacket){
  //     //LOG_sendBlocking(buffer_RobotPIDGains[id].packet.payload, REM_PACKET_SIZE_REM_ROBOT_PIDGAINS);
  //     buffer_RobotPIDGains[id].isNewPacket = false;
  //   }
  // }

  if (flagHandleConfiguration) {
    // TODO: Make a nice function for this
    REM_BasestationConfiguration configuration;
    configuration.header = REM_PACKET_TYPE_REM_BASESTATION_CONFIGURATION;
    configuration.remVersion = REM_LOCAL_VERSION;
    configuration.channel = Wireless_getChannel(SX_TX);

    REM_BasestationConfigurationPayload payload;
    encodeREM_BasestationConfiguration(&payload, &configuration);

    //LOG_sendBlocking(payload.payload, REM_PACKET_SIZE_REM_BASESTATION_CONFIGURATION);
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
  if(packet_rem_version != REM_LOCAL_VERSION){
    //LOG_printf("[handleRobotCommand] Error! packet_rem_version %u != %u REM_LOCAL_VERSION.", packet_rem_version, REM_LOCAL_VERSION);
    return;
  }

  // Store the message in the RobotCommand buffer. Set flag indicating packet needs to be sent to the robot
  uint8_t robot_id = REM_RobotCommand_get_toRobotId((REM_RobotCommandPayload*) packet_buffer);
  memcpy(/*buffer_REM_RobotCommand[robot_id].packet.payload*/ NULL, packet_buffer, REM_PACKET_SIZE_REM_ROBOT_COMMAND);
  buffer_REM_RobotCommand[robot_id].isNewPacket = true;
  // buffer_REM_RobotCommand[robot_id].counter++;
}

void handleRobotSetPIDGains(uint8_t* packet_buffer){
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
  if(packet_rem_version != REM_LOCAL_VERSION){
    //LOG_printf("[handleRobotFeedback] Error! packet_rem_version %u != %u REM_LOCAL_VERSION.", packet_rem_version, REM_LOCAL_VERSION);
    return;
  }

  // Store the message in the RobotFeedback buffer. Set flag indicating packet needs to be sent to the robot
  uint8_t robot_id = REM_RobotFeedback_get_toRobotId((REM_RobotFeedbackPayload*) packet_buffer);
  memcpy(buffer_REM_RobotFeedback[robot_id].packet.payload, packet_buffer, REM_PACKET_SIZE_REM_ROBOT_FEEDBACK);
  buffer_REM_RobotFeedback[robot_id].isNewPacket = true;
  // buffer_REM_RobotFeedback[robot_id].counter++;
}


/**
 * @brief Receives a buffer which is assumed to be holding a RobotStateInfo packet. 
 * If so, it moves the packet to the RobotStateInfo buffer, and sets a flag to send the packet to
 * the computer.
 * 
 * @param packet_buffer Pointer to the buffer that holds the packet
 */
void handleRobotStateInfo(uint8_t* packet_buffer){
}


/**
 * @brief Receives a buffer which is assumed to be holding a RobotBuzzer packet. 
 * If so, it moves the packet to the RobotBuzzer buffer, and sets a flag to send the packet to
 * the corresponding robot.
 * 
 * @param packet_buffer Pointer to the buffer that holds the packet
 */
void handleRobotBuzzer(uint8_t* packet_buffer){
}

void handleRobotGetPIDGains(uint8_t* packet_buffer){
}

void handleRobotPIDGains(uint8_t* packet_buffer){
}

void handleRobotMusicCommand(uint8_t* packet_buffer){
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
 * @param packets_buffer Pointer to the buffer the packet from the USB is stored in
 * @param packets_buffer_length Length of the packets currently in the buffer
 * @return true if the packet(s) have been handled succesfully
 * @return false if the packet(s) have been handled unsuccessfully, e.g. due to corruption
 */
bool handlePackets(uint8_t* packets_buffer, uint32_t packets_buffer_length){

  if(packets_buffer_length < 5){
    handled_INVALID++;
    handled_INVALID_nbytes += packets_buffer_length;
    memcpy(INVALID_buffer + handled_INVALID_nbytes, packets_buffer, packets_buffer_length);
    return true;
  }

  uint32_t bytes_processed = 0;

  while(bytes_processed < packets_buffer_length){

    // Get some information about the packet
    REM_PacketPayload* packet = (REM_PacketPayload*) packets_buffer;
    uint8_t packet_type = REM_Packet_get_header(packet);
    uint32_t packet_size = REM_Packet_get_payloadSize(packet);
    uint8_t packet_rem_version = REM_Packet_get_remVersion(packet);

    if(packet_rem_version != REM_LOCAL_VERSION){
      // Something something error, maybe. Maybe only error if the packet is meant for the basestation
    }

    // Check size of static packets. Should not be needed but can't hurt. Skip the LOG packet, since it has a dynamic length
    if(packet_type != REM_PACKET_TYPE_REM_LOG){
      if(packet_size != REM_PACKET_TYPE_TO_SIZE(packet_type)){
        // Something something error. We're now unsure how mamy bytes we have to transmit..
      }
    }

    // Figure out where the packet is headed to
    // If neither toPC or toBS is true, then that means that the packet is meant for a robot
    bool to_PC = REM_Packet_get_toPC(packet);  // Packet is destined for the PC
    bool to_BS = REM_Packet_get_toBS(packet);  // Packet is destined for the BaseStation
    bool to_robot = !(to_PC || to_BS);         // Packet is destined for a robot
    bool robot_id = REM_Packet_get_toRobotId(packet);

    // High priority : Deal with RobotCommand packets that are destined for a robot
    if(packet_type == REM_PACKET_TYPE_REM_ROBOT_COMMAND && to_robot){
      // Store the message in the RobotCommand buffer. Set flag indicating packet needs to be sent to the robot
      memcpy(buffer_REM_RobotCommand[robot_id].packet.payload, packets_buffer, packet_size);
      buffer_REM_RobotCommand[robot_id].isNewPacket = true;
    }else

    // High priority : Deal with RobotFeedback packets that are destined for the PC
    if(packet_type == REM_PACKET_TYPE_REM_ROBOT_FEEDBACK && to_PC){
      // Store the message in the RobotFeedback buffer. Set flag indicating packet needs to be sent to the PC
      memcpy(buffer_REM_RobotFeedback[robot_id].packet.payload, packets_buffer, packet_size);
      buffer_REM_RobotFeedback[robot_id].isNewPacket = true;
    }else
    // If filler packet is detected just continue
    if(packet_type == REM_PACKET_TYPE_REM_SX1280FILLER){
      bytes_processed+= REM_PACKET_SIZE_REM_SX1280FILLER;
      continue;
    }
    // Low priority : Deal with any other packet
    else{
      CircularBuffer* queue;
      uint8_t* buffer;
      if(to_robot){
        queue = nonpriority_queue_robots_index[robot_id];
        buffer = nonpriority_queue_robots[robot_id];
        if(CircularBuffer_canWrite(queue, 1)){
          memcpy(buffer[queue->indexWrite], packets_buffer, REM_PACKET_TYPE_TO_SIZE(packet_type));
          CircularBuffer_write(queue, NULL, 1);
        }
      }
      if(to_PC){
        if(CircularBuffer_canWrite(nonpriority_queue_pc_index, 1)){
          memcpy(&nonpriority_queue_pc[nonpriority_queue_pc_index->indexWrite], packets_buffer, REM_PACKET_TYPE_TO_SIZE(packet_type));
          CircularBuffer_write(nonpriority_queue_pc_index, NULL, 1);
        }
      }
      if(to_BS){
        if(CircularBuffer_canWrite(nonpriority_queue_bs_index, 1)){
          memcpy(&nonpriority_queue_bs[nonpriority_queue_bs_index->indexWrite], packets_buffer, REM_PACKET_TYPE_TO_SIZE(packet_type));
          CircularBuffer_write(nonpriority_queue_bs_index, NULL, 1);
        }
      }
    }

    bytes_processed += packet_size;

    //LOG_printf("[handlePackets] Error! At %ld of %ld bytes. [@] = %d\n", bytes_processed, packets_buffer_length, packets_buffer[bytes_processed]);

  }

  return true;
}



/* Triggers when a call to HAL_SPI_TransmitReceive_DMA or HAL_SPI_TransmitReceive_IT (both non-blocking) completes */
void HAL_SPI_TxRxCpltCallback(SPI_HandleTypeDef *hspi){
  if(hspi->Instance == SX_TX->Interface->SPI->Instance){
    Wireless_DMA_Handler(SX_TX);
    // SX_TX should never receive a packet so that's why we don't call handlePackets here.
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
    static uint8_t robot_id = 0;

    // Keeps track of the total length of the packet that goes to the robot. 
    // Cannot exceed MAX_PACKET_SIZE, or it will overflow the internal buffer of the SX1280
    uint8_t total_packet_length = 0;    

    /* Add RobotCommand to the transmission */
    if(buffer_REM_RobotCommand[robot_id].isNewPacket 
    && total_packet_length + REM_PACKET_SIZE_REM_ROBOT_COMMAND < MAX_PACKET_SIZE){
      buffer_REM_RobotCommand[robot_id].isNewPacket = false;
      memcpy(txPacket.message + total_packet_length, buffer_REM_RobotCommand[robot_id].packet.payload, REM_PACKET_SIZE_REM_ROBOT_COMMAND);
      total_packet_length += REM_PACKET_SIZE_REM_ROBOT_COMMAND;
    }

    /* Add any other packet from the queue to the transmission */
    CircularBuffer* queue = nonpriority_queue_robots[robot_id];
    while(true){
      // If mo more packets in queue to send, then break
      if(CircularBuffer_spaceFilled(queue) == 0) break;
      
    }

    /* Send new command if available for this robot ID */
    if(0 < total_packet_length){
      if(SX_TX->state == WIRELESS_READY){
        /* Add a filler packet to the buffer if there are currently less than 6 bytes in the buffer
        * The minimum payload size for the SX1280 in FLRC mode is 6 bytes. 
        * See documentation page 124 - Table 14-36: Sync Word Combination in FLRC Packet */
        if(total_packet_length < 6){
          memcpy(txPacket.message + total_packet_length, SX1280_filler_payload.payload, REM_PACKET_SIZE_REM_SX1280FILLER);
          total_packet_length += REM_PACKET_SIZE_REM_SX1280FILLER;
        }

        txPacket.payloadLength = total_packet_length;
        Wireless_setTXSyncword(SX_TX,robot_syncWord[robot_id]);
        WritePacket_DMA(SX_TX, &txPacket, &Wireless_Writepacket_Cplt);
      }
    }

    // Schedule next ID to be sent
    robot_id++;
    // Wrap around if the last ID has been dealt with
    if(MAX_ROBOT_ID < robot_id){
      robot_id = 0;
    }
  }
}