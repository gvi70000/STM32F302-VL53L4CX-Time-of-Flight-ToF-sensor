/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
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
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "dma.h"
#include "i2c.h"
#include "usart.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "VL53.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
// Sample calibration data including <> and check sum
#ifdef DEBUG
const uint8_t calData[392] = {0x3C, 0x53, 0x22, 0x01, 0xAB, 0xEC, 0xFF, 0xF5, 0xFF, 0xFF, 0xFF, 0x0B, 0x00, 0x06, 0x01, 0x00, 0x00, 0x00, 0x01, 0x10, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
	0x00, 0x0A, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0xE0, 0x1E, 0x78, 0xB3, 0x80, 0x00, 0x80, 0x01, 0x8C, 0x6F, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x0C, 0x0C,
	0x00, 0x02, 0x00, 0x00, 0x00, 0x18, 0x00, 0x00, 0x00, 0x70, 0x03, 0x00, 0x00, 0x76, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
	0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x4E, 0x28, 0x06, 0x09, 0x28, 0x00, 0x51, 0xBC, 0x4E, 0x10, 0x00, 0x00, 0x03,
	0x03, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x0C, 0x0C, 0x00, 0x00, 0x01, 0x02, 0x03, 0x04, 0x05, 0x01, 0x01, 0x01, 0x01, 0x01, 0x01, 0x17, 0x00, 0x00, 0x00, 0x12, 0x01, 0x00, 0x00,
	0x4C, 0x27, 0x00, 0x00, 0x45, 0x05, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
	0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
	0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
	0x00, 0x4E, 0x28, 0x06, 0x09, 0x28, 0x00, 0x00, 0x00, 0x51, 0xBC, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
	0x4E, 0x10, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0xDB, 0x07, 0xE4, 0x07, 0x00, 0x00, 0x00, 0x00, 0x19, 0x00, 0x05, 0x00, 0x05, 0x00, 0x00,
	0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
	0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0xF0, 0x00, 0xF2, 0x00, 0xF3, 0x00, 0xF3, 0x00, 0x01,
	0x10, 0x00, 0x00, 0xC3, 0x1F, 0x00, 0x00, 0x85, 0x2F, 0x00, 0x00, 0x47, 0x3F, 0x00, 0x00, 0x09, 0x4F, 0x00, 0x00, 0xCF, 0x5E, 0x00, 0x00, 0x1F, 0x3E};
#endif
// 388 bytes for calibration, 1 byte for C(alibration), 1 byte checksum, 2 bytes start/end message marker
volatile uint8_t rxBuffer[RX_BUFFER_LEN];

extern volatile uint8_t ToF_EventDetected;
extern VL53LX_CalibrationData_t calibrationData;

extern DMA_HandleTypeDef hdma_usart1_rx;
extern DMA_HandleTypeDef hdma_usart1_tx;

volatile uint8_t txNotComplete = 0;

uint32_t sumDist = 0;
uint8_t cntDist = 0;
uint16_t avgDist = 0;

uint8_t checkSum = 0;
uint8_t crtCommand = 0;

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

// Function to generate checksum and place it in the last position of the array
void generateChecksum(uint8_t *array, uint16_t length) {
	checkSum = 0;
	if (length < 3) return; // Ensure there's enough length for start, char, and checksum
	for (uint16_t i = 1; i < length - 1; i++) {
		checkSum += array[i];
	}
	array[length - 1] = checkSum; // Store the checksum in the last position
}

// Function to check if the checksum in the last position matches the calculated checksum
uint8_t checkChecksum(uint16_t length) {
	if (length < 4) return 0; // Not a valid structure
	uint8_t len = length - 2;
	uint8_t checksum = 0;
	for (uint16_t i = 1; i < len; i++) {
		checksum += rxBuffer[i];
	}
	return (checksum == rxBuffer[len]); // Compare calculated checksum with the stored one
}

void processMessage(const uint16_t msgLen) {
	// If checkSum is OK
	if(checkChecksum(msgLen)) {
		// Do we have the markers in place?
		if((rxBuffer[0] == START_MARKER) && (rxBuffer[msgLen - 1] == END_MARKER)) {
			switch(rxBuffer[1]) {
				case CMD_CALIBRATE:
					crtCommand = CMD_CALIBRATE;
				break;
				case CMD_STORE_CAL:
					crtCommand = CMD_STORE_CAL;
					calibrationData = *(VL53LX_CalibrationData_t *)&rxBuffer[2];
				break;				
			}
		}
	}
}

// DMA transmit complete callback
void HAL_UART_TxCpltCallback(UART_HandleTypeDef *huart){
	if(huart->Instance == USART1) {
		txNotComplete = 0;
	}
}

// DMA receive complete callback
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart) {
	if(huart->Instance == USART1) {
		//Determine how many items of data have been received
		uint8_t data_length = RX_BUFFER_LEN - __HAL_DMA_GET_COUNTER(huart1.hdmarx);//DMA1_Stream0->NDTR;
		//Stop DMA	
		HAL_UART_DMAStop(&huart1);
		// Shortest messege is <CDDC>
		if(data_length > 5) {
			processMessage(data_length);
		}
		HAL_UART_Receive_DMA(&huart1, (uint8_t *)rxBuffer, RX_BUFFER_LEN);
		__HAL_DMA_DISABLE_IT(&hdma_usart1_rx, DMA_IT_HT);
		__HAL_UART_ENABLE_IT(&huart1, UART_IT_IDLE);

	}
}

// Send messages to ESP
void sendMessage(const uint8_t msgType) {
	uint16_t txLen = 0;
	uint8_t txBuffer[RX_BUFFER_LEN];
	switch(msgType) {
		case RESPONSE_OK:
		case RESPONSE_FAIL:
			txLen = 4;
		break;	
		case RESPONSE_LENGTH:
			txLen = 6;
			*(uint16_t *)&txBuffer[2] = avgDist;
		break;
		case RESPONSE_CAL_DONE:
			txLen = RX_BUFFER_LEN;
			// Put the calibrationData structure in the array
			*(VL53LX_CalibrationData_t *)&txBuffer[2] = calibrationData;
		break;		
	}
	txBuffer[0] = START_MARKER;
	txBuffer[1] = msgType;
	generateChecksum(txBuffer, txLen - 1);
	txBuffer[txLen - 1] = END_MARKER;
	
	uint32_t startTime = HAL_GetTick();
	txNotComplete = 1; // Assume transmission is not complete
	HAL_UART_Transmit_DMA(&huart1, (uint8_t *)txBuffer, txLen);
	while(txNotComplete) {
		// Check for timeout
		if (HAL_GetTick() - startTime > TIMEOUT_SEND) {
			// Timeout occurred, abort transmission
			HAL_UART_AbortTransmit(&huart1);
			// Re-initialize UART
			MX_USART1_UART_Init(); 
			txNotComplete = 0; // Clear the transmission flag to exit the loop
			break; // Exit the loop
		}
	}
}
/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{
  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

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
  MX_DMA_Init();
  MX_I2C1_Init();
  MX_USART1_UART_Init();
  /* USER CODE BEGIN 2 */
	
	// Allow ESP to BOOT
	HAL_Delay(500);
	// Send a message to ESP32 to let it know that it can send the calibration data
	sendMessage(RESPONSE_OK);

	// Used to get the time needed for calibration = 4.9s
//	#ifdef DEBUG
//	VL53_Calibrate();
//	sendMessage(RESPONSE_CAL_DONE);
//	#endif

	// Simulate a calibration data received not using memcpy
//	for(uint16_t i = 0; i < RX_BUFFER_LEN; i++){
//		rxBuffer[i] = calData[i];
//	}
//	processMessage(RX_BUFFER_LEN);
	
	// Wait to get the calibrarion data from ESP
	uint8_t isTimeOut = 1;
	uint32_t time = HAL_GetTick();
	while(HAL_GetTick() - time < TIMEOUT_ESP) {
		if(crtCommand == CMD_STORE_CAL){
			isTimeOut = 0;
			break;
		}
		HAL_Delay(10);
	}
	// If we do not get anything
	if(isTimeOut) {
		sendMessage(RESPONSE_FAIL);
	} else {
		// Reset the command
		crtCommand = 0;
		// Inform ESP that we got the packet
		sendMessage(RESPONSE_OK);
	}
	// Start VL
	VL53_Init(isTimeOut);
	
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
		// Check if a calibration request was issued
		if(crtCommand == CMD_CALIBRATE){
			// If the calibration NOK. Calibration takes about 5sec to complete
			if(VL53_Calibrate() != VL53LX_ERROR_NONE) {
				sendMessage(RESPONSE_FAIL);
			} else {
				// Hard reset the sensor at this point
				sendMessage(RESPONSE_CAL_DONE);
				// Re init the sensor considering that we have calibration data available
				VL53_Init(0);
			}
			crtCommand = 0;
		}
		// Chek if distance data is available
		if (ToF_EventDetected) {
			uint16_t crtDist = VL53_getDistance();
			if(crtDist) { // Check if a valid distance is returned
				sumDist += crtDist;
				cntDist++;
				if(cntDist == AVG_COUNTER) { // Check if we have collected 25 readings
					avgDist = sumDist / AVG_COUNTER; // Calculate the average
					sendMessage(RESPONSE_LENGTH); // Send length to ESP
					sumDist = 0; // Reset the sum
					cntDist = 0; // Reset the counter
				}
			}
		}
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
  }
  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};
  RCC_PeriphCLKInitTypeDef PeriphClkInit = {0};

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI|RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.HSEPredivValue = RCC_HSE_PREDIV_DIV1;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL9;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
  {
    Error_Handler();
  }
  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_USART1|RCC_PERIPHCLK_I2C1;
  PeriphClkInit.Usart1ClockSelection = RCC_USART1CLKSOURCE_PCLK2;
  PeriphClkInit.I2c1ClockSelection = RCC_I2C1CLKSOURCE_HSI;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
  {
    Error_Handler();
  }
}

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  __disable_irq();
  while (1)
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
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
