/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2020 STMicroelectronics.
  * All rights reserved.</center></h2>
  *
  * This software component is licensed by ST under Ultimate Liberty license
  * SLA0044, the "License"; You may not use this file except in compliance with
  * the License. You may obtain a copy of the License at:
  *                             www.st.com/SLA0044
  *
  ******************************************************************************
  */
/* USER CODE END Header */

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __MAIN_H
#define __MAIN_H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "stm32f7xx_hal.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Exported types ------------------------------------------------------------*/
/* USER CODE BEGIN ET */

/* USER CODE END ET */

/* Exported constants --------------------------------------------------------*/
/* USER CODE BEGIN EC */

/* USER CODE END EC */

/* Exported macro ------------------------------------------------------------*/
/* USER CODE BEGIN EM */

/* USER CODE END EM */

/* Exported functions prototypes ---------------------------------------------*/
void Error_Handler(void);

/* USER CODE BEGIN EFP */

/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/
#define APB1 108
#define APB2 216
#define USER_Btn_Pin GPIO_PIN_13
#define USER_Btn_GPIO_Port GPIOC
#define USER_Btn_EXTI_IRQn EXTI15_10_IRQn
#define MCO_Pin GPIO_PIN_0
#define MCO_GPIO_Port GPIOH
#define LD_LED3_Pin GPIO_PIN_1
#define LD_LED3_GPIO_Port GPIOC
#define LD_ACTIVE_Pin GPIO_PIN_0
#define LD_ACTIVE_GPIO_Port GPIOA
#define LD_USB_Pin GPIO_PIN_1
#define LD_USB_GPIO_Port GPIOA
#define LD_LED1_Pin GPIO_PIN_4
#define LD_LED1_GPIO_Port GPIOA
#define LD_LED2_Pin GPIO_PIN_0
#define LD_LED2_GPIO_Port GPIOB
#define SX_RX_IRQ_Pin GPIO_PIN_7
#define SX_RX_IRQ_GPIO_Port GPIOE
#define SX_RX_IRQ_EXTI_IRQn EXTI9_5_IRQn
#define SX_RX_RST_Pin GPIO_PIN_10
#define SX_RX_RST_GPIO_Port GPIOE
#define RMII_TXD1_Pin GPIO_PIN_13
#define RMII_TXD1_GPIO_Port GPIOB
#define LD3_Pin GPIO_PIN_14
#define LD3_GPIO_Port GPIOB
#define STLK_RX_Pin GPIO_PIN_8
#define STLK_RX_GPIO_Port GPIOD
#define STLK_TX_Pin GPIO_PIN_9
#define STLK_TX_GPIO_Port GPIOD
#define SX_RX_BUSY_Pin GPIO_PIN_11
#define SX_RX_BUSY_GPIO_Port GPIOD
#define LED_RX_Pin GPIO_PIN_4
#define LED_RX_GPIO_Port GPIOG
#define USB_PowerSwitchOn_Pin GPIO_PIN_6
#define USB_PowerSwitchOn_GPIO_Port GPIOG
#define USB_OverCurrent_Pin GPIO_PIN_7
#define USB_OverCurrent_GPIO_Port GPIOG
#define LED_TX_Pin GPIO_PIN_9
#define LED_TX_GPIO_Port GPIOC
#define USB_SOF_Pin GPIO_PIN_8
#define USB_SOF_GPIO_Port GPIOA
#define USB_VBUS_Pin GPIO_PIN_9
#define USB_VBUS_GPIO_Port GPIOA
#define USB_DM_Pin GPIO_PIN_11
#define USB_DM_GPIO_Port GPIOA
#define USB_DP_Pin GPIO_PIN_12
#define USB_DP_GPIO_Port GPIOA
#define TMS_Pin GPIO_PIN_13
#define TMS_GPIO_Port GPIOA
#define TCK_Pin GPIO_PIN_14
#define TCK_GPIO_Port GPIOA
#define SX_TX_RST_Pin GPIO_PIN_10
#define SX_TX_RST_GPIO_Port GPIOC
#define SX_TX_BUSY_Pin GPIO_PIN_11
#define SX_TX_BUSY_GPIO_Port GPIOC
#define SX_TX_IRQ_Pin GPIO_PIN_2
#define SX_TX_IRQ_GPIO_Port GPIOD
#define SX_TX_IRQ_EXTI_IRQn EXTI2_IRQn
#define SX_RX_CS_Pin GPIO_PIN_5
#define SX_RX_CS_GPIO_Port GPIOD
#define SWO_Pin GPIO_PIN_3
#define SWO_GPIO_Port GPIOB
#define LD2_Pin GPIO_PIN_7
#define LD2_GPIO_Port GPIOB
#define SX_TX_CS_Pin GPIO_PIN_9
#define SX_TX_CS_GPIO_Port GPIOB
/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
