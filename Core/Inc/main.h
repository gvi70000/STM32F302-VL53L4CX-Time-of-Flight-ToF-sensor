/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2024 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
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
#include "stm32f3xx_hal.h"

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
//#define DEBUG
#define TIMEOUT_SEND 1000
#define TIMEOUT_ESP	 6000
// 388 bytes for calibration, 1 byte for C(alibration), 1 byte checksum, 2 bytes start/end message marker
#define RX_BUFFER_LEN			392
#define START_MARKER			60 // < Start marker for serial
#define END_MARKER				62 // > End marker for serial
#define CMD_CALIBRATE			67 // Get C to start calibration
#define CMD_STORE_CAL			83 // Get S to store calibration data receive from ESP

#define RESPONSE_LENGTH		76 // Send L to inform about length data
#define RESPONSE_CAL_DONE	CMD_CALIBRATE // Send C to inform about calibration data
#define RESPONSE_FAIL			70 // Send F to indicate something went wrong
#define RESPONSE_OK				CMD_STORE_CAL // Send S to indicate something went OK
/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/
#define INT_Pin GPIO_PIN_8
#define INT_GPIO_Port GPIOB
#define INT_EXTI_IRQn EXTI9_5_IRQn
#define XSHUT_Pin GPIO_PIN_9
#define XSHUT_GPIO_Port GPIOB

/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
