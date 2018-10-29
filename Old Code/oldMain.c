
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  ** This notice applies to any and all portions of this file
  * that are not between comment pairs USER CODE BEGIN and
  * USER CODE END. Other portions of this file, whether 
  * inserted by the user or by software development tools
  * are owned by their respective copyright owners.
  *
  * COPYRIGHT(c) 2018 STMicroelectronics
  *
  * Redistribution and use in source and binary forms, with or without modification,
  * are permitted provided that the following conditions are met:
  *   1. Redistributions of source code must retain the above copyright notice,
  *      this list of conditions and the following disclaimer.
  *   2. Redistributions in binary form must reproduce the above copyright notice,
  *      this list of conditions and the following disclaimer in the documentation
  *      and/or other materials provided with the distribution.
  *   3. Neither the name of STMicroelectronics nor the names of its contributors
  *      may be used to endorse or promote products derived from this software
  *      without specific prior written permission.
  *
  * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
  * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
  * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
  * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
  * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
  * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
  * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
  * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
  * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
  * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
  *
  ******************************************************************************
  */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "stm32f4xx_hal.h"
#include "i2c.h"
#include "spi.h"
#include "tim.h"
#include "usart.h"
#include "gpio.h"

/* USER CODE BEGIN Includes */
#include <math.h>
#include <stdlib.h>
#include "PuttyInterface/PuttyInterface.h"
#include "userIO/userIO.h"
#include "Geneva/geneva.h"
#include "DO/DO.h"
#include "Ballsensor/ballsensor.h"
//#include "myNRF24.h"
#include "wheels/wheels.h"
#include "kickchip/kickchip.h"
#include "MTi/MTiControl.h"
#include "wireless/wireless.h"
#include "dribbler/dribbler.h"

/* USER CODE END Includes */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
/* Private variables ---------------------------------------------------------*/
#define STOP_AFTER 250 //ms
PuttyInterfaceTypeDef puttystruct;
bool battery_empty = false;
bool user_control = false;
bool print_encoder = false;
bool print_euler = false;
bool wheels_testing = false;
float wheels_testing_power = 3000;
bool keyboard_control = false;
bool started_icc = false;
bool halt = true;
bool calibration_needed = true;
bool vision_available = false;


//float wheelsPWM[4] = {0};
float velocityRef[3] = {0};
float vision_yaw = 0;
uint kick_timer = 0;
uint8_t corrected_kick_power = 0;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);

/* USER CODE BEGIN PFP */
/* Private function prototypes -----------------------------------------------*/
void HandleCommand(char* input);

/* USER CODE END PFP */

/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  *
  * @retval None
  */
int main(void)
{
  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MCU Configuration----------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_TIM8_Init();
  MX_TIM4_Init();
  MX_TIM3_Init();
  MX_TIM2_Init();
  MX_TIM9_Init();
  MX_TIM12_Init();
  MX_TIM10_Init();
  MX_TIM11_Init();
  MX_USART2_UART_Init();
  MX_I2C1_Init();
  MX_SPI2_Init();
  MX_USART3_UART_Init();
  MX_TIM1_Init();
  MX_TIM6_Init();
  MX_TIM7_Init();
  MX_TIM5_Init();
  MX_TIM13_Init();
  MX_TIM14_Init();
  /* USER CODE BEGIN 2 */
  puttystruct.handle = HandleCommand;
  PuttyInterface_Init(&puttystruct);
  DO_Init();
  dribbler_Init();
  ballsensorInit();
  wheels_Init();
  MT_Init();
  geneva_Init();
  kick_Init();

  if(HAL_GPIO_ReadPin(SW_freq_GPIO_Port, SW_freq_Pin)){
	  Wireless_Init(ReadAddress(), 103);
  }else{
	  Wireless_Init(ReadAddress(), 80);
  }

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  uint printtime = 0;
  uint battery_count = 0;
  preparedAckData.batteryState = 0;
  enable_acks = 0;
  while (1)
  {
	HAL_GPIO_TogglePin(Switch_GPIO_Port,Switch_Pin);

	if(Wireless_newData()) {
		Wireless_newPacketHandler();
		//printBallPosition();
		//HAL_GPIO_TogglePin(LD3_GPIO_Port, LD3_Pin);

		if (!calibration_needed) {
			  halt = false; // robot is only allowed to move after packages are received and yaw calibration is not needed
		}
		LastPackageTime = HAL_GetTick();

		//TODO: test this data coming in
		float velRefAmp = (float)receivedRoboData.rho * 0.004F;
		float velRefDir = (float)receivedRoboData.theta / 1024.0F * M_PI;
		float angularVelRef = (float)receivedRoboData.velocity_angular / 512.0F * 16.0F*M_PI;
		velocityRef[body_x] = cosf(velRefDir) * velRefAmp;
		velocityRef[body_y] = sinf(velRefDir) * velRefAmp;
		if(receivedRoboData.use_angle){
			velocityRef[body_w] = angularVelRef;
		}

		//TODO: test vision angle and calibration etc.
		vision_available = receivedRoboData.use_cam_info;
		if (vision_available) {
			vision_yaw = ((float)receivedRoboData.cam_rotation/1024.0F)*M_PI;
		}

		//dribbler
		dribbler_SetSpeed(receivedRoboData.velocity_dribbler);

		//kicker
		float kick_power = (float)receivedRoboData.kick_chip_power/255.0F*8.0F; // represented kick power
		if (kick_power > 4.5F) {
			corrected_kick_power = 99.0; // above a certain command, we cannot shoot any faster, so we choose maximum power for certainty
		} else {
			corrected_kick_power = (kick_power*0.5F+1.0F)/8.0F*100.0F; // scale (0 to 8) range to (1 to 5) range such that in software the command matches the start velocity of the ball
		}
//		uprintf("[%f, %f, %i]\n\r", kick_power, (float)(corrected_kick_power)/100*8,corrected_kick_power);
		if (receivedRoboData.kick_chip_forced/* && ((HAL_GetTick() - kick_timer) > 0)*/){
			kick_timer = HAL_GetTick() + 1000U;
			float kick_power = (float)receivedRoboData.kick_chip_power/255.0F*8.0F; // represented kick power
			float corrected_kick_power;
			if (kick_power > 4.5F) {
				corrected_kick_power = 99; // above a certain command, we cannot shoot any faster, so we choose maximum power for certainty
			} else {
				corrected_kick_power = (kick_power*0.5F+1.0F)/8.0F*100.0F; // scale (0 to 8) range to (1 to 5) range such that in software the command matches the start velocity of the ball
			}
			kick_Shoot(corrected_kick_power,!receivedRoboData.do_chip);
		}

		//geneva
		if (receivedRoboData.geneva_drive_state != 0){
			geneva_SetPosition(receivedRoboData.geneva_drive_state-1);
		}

	}else if(wheels_testing){
		velocityRef[body_w] = wheels_testing_power;
		halt = false;
	}else if((HAL_GetTick() - LastPackageTime > STOP_AFTER)/* && !user_control*/){; // if no new wireless data
		halt = true;
		vision_available = false;
	}
	// check if battery is empty
	if(HAL_GPIO_ReadPin(empty_battery_GPIO_Port, empty_battery_Pin)){
		if(battery_count++ == 1000){
			uprintf("Battery empty!\n\r");
			ToggleLD(4);
			wheels_DeInit();
			kick_DeInit();
			dribbler_Deinit();
			geneva_Deinit();
			preparedAckData.batteryState = 1;
			Wireless_Deinit();
		}
	}


	preparedAckData.ballSensor = ballsensorMeasurementLoop(receivedRoboData.do_kick, receivedRoboData.do_chip, corrected_kick_power);
	if(preparedAckData.ballSensor == NOBALL){
		SetLD(2, 0);
	}else{
		SetLD(2, 1);
	}
//	preparedAckData.ballSensor = ballsensorMeasurementLoop(1, receivedRoboData.do_chip, 30);

	//uprintf("ball: %i\n",preparedAckData.ballSensor);

	geneva_Update();
	MT_Update();
	if((HAL_GetTick() - printtime > 1000)){
		printtime = HAL_GetTick();
		ToggleLD(1);

		//CHECK NRF STATUS REGISTER EVERY SECOND...IF NRF IS CONSTIPATED, FLUSH SHIT OUT
		if(readReg(STATUS) != 0x0e) {
			nrfPrintStatus(readReg(STATUS));
			uprintf("--> Flushing constipated NRF\n");
			flushRX();
			clearInterrupts();
		}

		if(*MT_GetStatusWord() & 0b00011000){
			//uprintf("calibrating Xsens.\n\r");
		}else if(started_icc == false){
			started_icc = true;
			if(MT_UseIcc() == MT_succes)
				uprintf("Xsens calibration done.\n\r");
		}
		//uprintf("ballSensor = [%d]\n\r", preparedAckData.ballSensor);
		//uprintf("MT status suc/err = [%u/%u]\n\r", MT_GetSuccErr()[0], MT_GetSuccErr()[1]);
		//uprintf("status word [%08lx]\n\r", (unsigned long)*MT_GetStatusWord());
		//uprintf("charge = %d\n\r", HAL_GPIO_ReadPin(Charge_GPIO_Port, Charge_Pin));
		//uprintf("geneva_state = [%d]\n\r", geneva_GetState());
	}
  /* USER CODE END WHILE */

  /* USER CODE BEGIN 3 */
	PuttyInterface_Update(&puttystruct); // Place this bloody thing into a fuc***g timer pls.
  } // end while loop
  /* USER CODE END 3 */

}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{

  RCC_OscInitTypeDef RCC_OscInitStruct;
  RCC_ClkInitTypeDef RCC_ClkInitStruct;

    /**Configure the main internal regulator output voltage 
    */
  __HAL_RCC_PWR_CLK_ENABLE();

  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

    /**Initializes the CPU, AHB and APB busses clocks 
    */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 4;
  RCC_OscInitStruct.PLL.PLLN = 192;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV4;
  RCC_OscInitStruct.PLL.PLLQ = 4;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

    /**Initializes the CPU, AHB and APB busses clocks 
    */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV4;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_3) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

    /**Configure the Systick interrupt time 
    */
  HAL_SYSTICK_Config(HAL_RCC_GetHCLKFreq()/1000);

    /**Configure the Systick 
    */
  HAL_SYSTICK_CLKSourceConfig(SYSTICK_CLKSOURCE_HCLK);

  /* SysTick_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(SysTick_IRQn, 0, 0);
}

/* USER CODE BEGIN 4 */
#define TEST_WHEELS_COMMAND "test wheels"
#define SET_FILTER_COMMAND "mt filter"
void HandleCommand(char* input){
	if(strcmp(input, "mt start") == 0){
		uprintf("Starting device MTi\n\r");
		if(MT_succes == MT_Init()){
			uprintf("MTi started.\n\r");
		}else{
			uprintf("No communication with MTi!\n\r");
		}
	}else if(!strcmp(input, "mt stop")){
		uprintf("resetting the MTi.\n\r");
		MT_DeInit();
	}else if(strcmp(input, "mt config") == 0){
		MT_GoToConfig();
	}else if(!strcmp(input, "mt measure")){
		MT_GoToMeasure();
	}else if(!strcmp(input, "mt options")){
		MT_ReqOptions();
	}else if(!strcmp(input, "mt setoptions")){
		MT_SetOptions();
	}else if(!strcmp(input, "mt icc")){
		MT_UseIcc();
	}else if(!strcmp(input, "mt norotation")){
		MT_NoRotation(10);
	}else if(!memcmp(input, SET_FILTER_COMMAND , strlen(SET_FILTER_COMMAND))){
		MT_SetFilterProfile(strtol(input + 1 + strlen(SET_FILTER_COMMAND), NULL, 10));
	}else if(strcmp(input, "mt factoryreset") == 0){
		uprintf("Resetting the configuration.\n\r");
		MT_FactoryReset();
	}else if(strcmp(input, "mt reqfilter") == 0){
		uprintf("requesting current filter profile.\n\r");
		MT_ReqFilterProfile();
	}else if(memcmp(input, "mt setconfig", strlen("mt setconfig")) == 0){
		MT_BuildConfig(XDI_PacketCounter, 100, false);
		MT_BuildConfig(XDI_FreeAcceleration, 100, false);
		MT_BuildConfig(XDI_EulerAngles, 100, true);
	}else if(strcmp(input, "reqconfig") == 0){
		uprintf("requesting output configuration mode\n\r");
		MT_RequestConfig();
	}else if(!strcmp(input, "address")){
		uprintf("address = [%d]\n\r", localRobotID);
	}else if(!strcmp(input, "example2")){
		uprintf("stop!\n\r");
	}else if(!strcmp(input, "geneva get")){
		uprintf("position = [%u]\n\r", geneva_GetPosition());
	}else if(!strcmp(input, "geneva stop")){
		geneva_SetState(geneva_idle);
	}else if(!strcmp(input, "euler")){
		print_euler = ! print_euler;
	}else if(!memcmp(input, "geneva set" , strlen("geneva set"))){
		geneva_SetPosition(2 + strtol(input + 1 + strlen("geneva set"), NULL, 10));
	}else if(!memcmp(input, "control" , strlen("control"))){
		geneva_SetPosition(2 + strtol(input + 1 + strlen("control"), NULL, 10));
	}else if(!memcmp(input, "kick" , strlen("kick"))){
		kick_Shoot(strtol(input + 1 + strlen("kick"), NULL, 10),KICK);
	}else if(!memcmp(input, "chip" , strlen("chip"))){
		kick_Shoot(strtol(input + 1 + strlen("chip"), NULL, 10),CHIP);
	}else if(!memcmp(input, "block" , strlen("block"))){
		kick_Stateprint();
	}else if(!memcmp(input, TEST_WHEELS_COMMAND, strlen(TEST_WHEELS_COMMAND))){
		wheels_testing_power = atoff(input + strlen(TEST_WHEELS_COMMAND));
		wheels_testing = (wheels_testing_power <= -10 || wheels_testing_power >= 10);
		if((wheels_testing)){
			uprintf("wheels test on, pwm [%f]\n\r", wheels_testing_power);
		}
	}else if(!memcmp(input, "dribble", strlen("dribble"))){
		uint8_t speed = strtol(input + strlen("dribble"), NULL, 10);
		dribbler_SetSpeed(speed);
		uprintf("speed is set to[%lu]\n\r", __HAL_TIM_GET_COMPARE(&htim11, TIM_CHANNEL_1));
	}

	else if(!strcmp(input, "keyboard control")){
		uprintf("going to keyboard control\r\npress escape to stop\n\r");
		keyboard_control = true;
		wheels_testing = true;
	}
}



void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart){
	// if(huart->Instance == huart3.Instance){//input from the PC
		// puttystruct.huart_Rx_len = 1;
		// puttystruct.small_buf[0] = *(huart->pRxBuffPtr-1);
	}else if(huart->Instance == huartMT.Instance){// Input from the Xsens
		MT_UART_RxCpltCallback();
	}
}

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef* htim){
	if(htim->Instance == htim6.Instance){
		geneva_Control();
	}else if(htim->Instance == htim7.Instance){
//		HAL_GPIO_WritePin(LD5_GPIO_Port,LD5_Pin, 1);
		float wheelsPWM[4] = {0,0,0,0};
		calibration_needed = DO_Control(velocityRef, vision_yaw, vision_available, wheelsPWM); // outputs to wheelsPWM
		if (calibration_needed) {
			halt = true;
		}
		 // send PWM to motors
		if (halt) { // when communication is lost for too long, we send 0 to the motors
			float wheel_powers[4] = {0,0,0,0};
			wheels_SetOutput(wheel_powers);
		} else {
			// copy the controller output before sending it to SetOutput, to make sure it doesnt get altered at the wrong time
			float wheel_powers[4] = {wheelsPWM[0],wheelsPWM[1],wheelsPWM[2],wheelsPWM[3]};
//			float wheel_powers[4] = {20,20,20,20};
			wheels_SetOutput(wheel_powers);
		}
//		HAL_GPIO_WritePin(LD5_GPIO_Port,LD5_Pin, 0);

//		fillXSensData(xsensData);

		//if(wheels_testing)	uprintf("wheels speeds are[%f %f %f %f]\n\r", wheels_GetSpeed(wheels_LF), wheels_GetSpeed(wheels_RF), wheels_GetSpeed(wheels_RB), wheels_GetSpeed(wheels_LB));
		//if(wheels_testing)	uprintf("wheels encoders are[%d %d %d %d]\n\r", wheels_GetEncoder(wheels_RF), wheels_GetEncoder(wheels_RB), wheels_GetEncoder(wheels_LB), wheels_GetEncoder(wheels_LF));
//		float * euler;
//		euler = MT_GetAngles();
//		if(Xsens_state == Xsens_Measure && print_euler)	uprintf("euler angles[%f, %f, %f]\n\r", euler[0], euler[1], euler[2]);
	}else if(htim->Instance == htim13.Instance){
		kick_Callback();
	}else if(htim->Instance == htim14.Instance){
		wheels_Callback();
	}
}

void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin){
	if(GPIO_Pin == SPI1_IRQ_Pin){
		//Wireless_newPacketHandler();
	}else if(GPIO_Pin == Geneva_cal_sens_Pin){
		// calibration  of the geneva drive finished
		//uprintf("geneva sensor\n\r");
		geneva_SensorCallback();
	}
}




/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @param  file: The file name as string.
  * @param  line: The line in file as a number.
  * @retval None
  */
void _Error_Handler(char *file, int line)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  while(1)
  {
  }
  /* USER CODE END Error_Handler_Debug */
}

#ifdef  USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t* file, uint32_t line)
{ 
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     tex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */

/**
  * @}
  */

/**
  * @}
  */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
