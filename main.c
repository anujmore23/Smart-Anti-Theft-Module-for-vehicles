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

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "string.h"
#include "stdio.h"
#include "stdbool.h"

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define DEBUG_MESSAGE_SIZE 200

/* GPS declarations ---------------------------------------------------------------------------------------------*/
#define GPS_RX_BUFFER_SIZE 600
#define GPS_TX_BUFFER_SIZE 600

/* SMS declarations ---------------------------------------------------------------------------------------------*/
#define SMS_RX_BUFFER_SIZE 100
#define MESSAGE_BUFFER_SIZE 100

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
TIM_HandleTypeDef htim16;

UART_HandleTypeDef huart1;
UART_HandleTypeDef huart2;
UART_HandleTypeDef huart3;
DMA_HandleTypeDef hdma_usart1_rx;
DMA_HandleTypeDef hdma_usart2_tx;

/* USER CODE BEGIN PV */
void extractGPRMCData(uint8_t *buffer);


int HAL_UART_ReceiveAvailable(UART_HandleTypeDef *huart);
int Extract_SMS_Number(uint8_t *sms_data);
void Clear_RX_Buffer(uint8_t *buffer, size_t size);
bool isArrayZero(const uint8_t *array, size_t size);
int Extract_SMS_Message(uint8_t *sms_data, char *output_message, size_t output_size);
void SendSMS(char *mobileNumber, char *message);



/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_USART1_UART_Init(void);
static void MX_USART3_UART_Init(void);
static void MX_TIM16_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
char debug_message[DEBUG_MESSAGE_SIZE];

/* GPS declarations ---------------------------------------------------------------------------------------------*/
uint8_t gps_rx_buffer[GPS_RX_BUFFER_SIZE]; // Buffer to store incoming GPS data
uint8_t gps_tx_buffer[GPS_TX_BUFFER_SIZE]; // Buffer to send GPS data to User
uint8_t location_data[300];




char time[15];                 // Buffer to store time (hhmmss.sss)
char latitude[20];             // Buffer to store latitude
char longitude[20];            // Buffer to store longitude
char status;                   // Status of the GPS fix (A or V)
char latitudeDirection[2];	   //Lat. Direction
char longitudeDirection[2];	   //Lon. Direction
char location_std_format[32]; // Location data in std format
char parked_location[32] = {1};
char temp_location_buffer[32] = {0};

char temp_time[15]; 		//test buffer to store temporary time for comparison with saved time
char parked_time[15]; 		//test buffer to store temporary time for comparison with saved time

/* SMS declarations ---------------------------------------------------------------------------------------------*/
uint8_t sms_rx_buffer[SMS_RX_BUFFER_SIZE];  // Buffer for receiving SMS

char mobileNumber[] = "+12163014356";  // Enter the Mobile Number you want to send to
//char formatted_buffer[SMS_RX_BUFFER_SIZE * 2]; // Buffer to hold the formatted string
char ATcommand[100];
char formatted_buffer[SMS_RX_BUFFER_SIZE * 2]; // Buffer to hold the formatted string


/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{

  /* USER CODE BEGIN 1 */
	int sms_number;
	char extracted_message[MESSAGE_BUFFER_SIZE];
	int message_sent;
	int IsParked;



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
  MX_USART2_UART_Init();
  MX_USART1_UART_Init();
  MX_USART3_UART_Init();
  MX_TIM16_Init();
  /* USER CODE BEGIN 2 */
  //Start Timer
  HAL_TIM_Base_Start(&htim16);
//---------------------------------------------------------------------------------------------------------
    sprintf(ATcommand, "AT+CMGF=1\r\n");
    HAL_UART_Transmit(&huart3, (uint8_t *)ATcommand, strlen(ATcommand), 1000);
    HAL_UART_Receive(&huart3, sms_rx_buffer, SMS_RX_BUFFER_SIZE, 100);
    HAL_Delay(100);

//---------------------------------------------------------------------------------------------------------
    sprintf(ATcommand, "AT+CNMI=?\r\n");
    HAL_UART_Transmit(&huart3, (uint8_t *)ATcommand, strlen(ATcommand), 1000);
    HAL_UART_Receive(&huart3, sms_rx_buffer, SMS_RX_BUFFER_SIZE, 100);
    HAL_Delay(100);
//---------------------------------------------------------------------------------------------------------
	sprintf(ATcommand, "AT+CMGD=,4\r\n");  // Delete all message in inbox
	HAL_UART_Transmit(&huart3, (uint8_t *)ATcommand, strlen(ATcommand), 1000);
	HAL_UART_Receive(&huart3, sms_rx_buffer, SMS_RX_BUFFER_SIZE, 100);

//---------------------------------------------------------------------------------------------------------

  HAL_UART_Receive_DMA(&huart1, gps_rx_buffer , GPS_RX_BUFFER_SIZE); // Receive Data from GPS Module and trigger callback after completed

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
	  	  memset(temp_location_buffer, 0, 32 );
	  	  message_sent = 0;
	  	  strcpy((char *)temp_time, (char *)time);								//temp buffer to store time for comparison
	  	  strcpy((char *)temp_location_buffer, (char *)location_std_format);	//temp buffer to store location for comparison

	  	  if(HAL_UART_ReceiveAvailable(&huart3))
	  	  {
			HAL_UART_Receive(&huart3, sms_rx_buffer, SMS_RX_BUFFER_SIZE, 100);
			sprintf(formatted_buffer, "Contents of sms_rx_buffer: %s\r\n", sms_rx_buffer);
			HAL_UART_Transmit(&huart2, (uint8_t *)formatted_buffer, strlen(formatted_buffer), 100);
	  	  }
		  if (isArrayZero(sms_rx_buffer, SMS_RX_BUFFER_SIZE))
		  {
				sprintf(formatted_buffer, "nothing in sms buffer\r\n");
				HAL_UART_Transmit(&huart2, (uint8_t *)formatted_buffer, strlen(formatted_buffer), 100);
		  }
		  else
		  {
			sms_number = Extract_SMS_Number(sms_rx_buffer); // Get sms number from ----- +CMTI: "SM",# ----- if doesnt exists, sms_number = -1

			if(sms_number != -1)
			{
				sprintf(ATcommand, "AT+CMGR=%d\r\n", sms_number);  // Delete all message in inbox
				HAL_UART_Transmit(&huart3, (uint8_t *)ATcommand, strlen(ATcommand), 100);
				HAL_UART_Receive(&huart3, sms_rx_buffer, SMS_RX_BUFFER_SIZE, 100);
				Extract_SMS_Message(sms_rx_buffer, extracted_message, MESSAGE_BUFFER_SIZE);
				sprintf(formatted_buffer, "extracted messages: %s\r\n", extracted_message);
				HAL_UART_Transmit(&huart2, (uint8_t *)formatted_buffer, strlen(formatted_buffer), 100);

				if (strstr(extracted_message, "Get Location") != NULL && !message_sent)
				{
					// Do something for "Get Location"
					SendSMS(mobileNumber, (char *)location_data );

					sprintf(formatted_buffer, "Location Information sent\r\n");
					HAL_UART_Transmit(&huart2, (uint8_t *)formatted_buffer, strlen(formatted_buffer), 100);
					message_sent = 1;

					  //timer_val = __HAL_TIM_GET_COUNTER(&htim16) - timer_val;
					  //uart_buf_len = sprintf(uart_buf, "%u us \r\n", timer_val);
					  //HAL_UART_Transmit(&huart2, (uint8_t *)uart_buf, uart_buf_len, 100);

				}
				else if (strstr(extracted_message, "Lock") != NULL && !message_sent)
				{
					// Do something for "Lock"

					strcpy((char *)parked_location, (char *)temp_location_buffer); // Store current location in parked location for comparison with continuously changing location
					strcpy((char *)parked_time, (char *)time);

					strcpy((char *)debug_message, (char *)parked_location); // Store current location in parked location for comparison with continuously changing location
					strncat((char *)debug_message, " is the Parked Location", sizeof(debug_message) - strlen((char *)debug_message) - 1);

					SendSMS(mobileNumber, debug_message );
					//HAL_Delay(1000);
					//SendSMS(mobileNumber, (char *)parked_location );

					IsParked = 1;
					message_sent = 1;
				}

				else if (strstr(extracted_message, "Immobilize") != NULL && !message_sent)
				{
					// Do something for "Led On"
					SendSMS(mobileNumber,"Car is Immobilized" );
					HAL_GPIO_WritePin(GPIOA, GPIO_PIN_5, GPIO_PIN_SET);
					sprintf(formatted_buffer, "Car is Immobilized\r\n");
					HAL_UART_Transmit(&huart2, (uint8_t *)formatted_buffer, strlen(formatted_buffer), 100);
					message_sent = 1;
				}
				else if (strstr(extracted_message, "Mobilize") != NULL && !message_sent)
				{
					// Do something for "Led Off"
					SendSMS(mobileNumber, "Car is Mobilized" );

					HAL_GPIO_WritePin(GPIOA, GPIO_PIN_5, GPIO_PIN_RESET);
					sprintf(formatted_buffer, "Car is mobilized\r\n");
					HAL_UART_Transmit(&huart2, (uint8_t *)formatted_buffer, strlen(formatted_buffer), 100);
					message_sent = 1;
				}
				else
				{
					// Handle unknown message
					sprintf(debug_message, "Unknown Message\r\n");
					HAL_UART_Transmit(&huart2, (uint8_t *)debug_message, DEBUG_MESSAGE_SIZE, 200);
				}
				if (sms_number >= 5)
				{
					sprintf(ATcommand, "AT+CMGD=,4\r\n");  // Delete all message in inbox
					HAL_UART_Transmit(&huart3, (uint8_t *)ATcommand, strlen(ATcommand), 100);
					HAL_UART_Receive(&huart3, sms_rx_buffer, SMS_RX_BUFFER_SIZE, 100);
					sprintf(debug_message, "All Messages Deleted\r\n");
					HAL_UART_Transmit(&huart2, (uint8_t *)debug_message, DEBUG_MESSAGE_SIZE, 100);
				}

			}

		  }
		  //memset(location_std_format, 0, sizeof(location_std_format)); // clear the message buffer or else same data of the buffer will be tx'd again and again.


		  if (IsParked)
			{
				int a = strcmp((char *)parked_location, (char *)temp_location_buffer);
				if(!a)
				{
					//Parked location and current location is same

				}
				else
				{
					sprintf(formatted_buffer, "Car is moving unauthorized !!!\r\n");
					HAL_UART_Transmit(&huart2, (uint8_t *)formatted_buffer, strlen(formatted_buffer), 200);
					SendSMS(mobileNumber, "Car is moving unauthorized !!!\r\n" );

					IsParked = 0;
				}
			}

	  	  memset(extracted_message, 0, MESSAGE_BUFFER_SIZE);
	  	  memset(debug_message, 0, DEBUG_MESSAGE_SIZE);

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

  /** Configure the main internal regulator output voltage
  */
  if (HAL_PWREx_ControlVoltageScaling(PWR_REGULATOR_VOLTAGE_SCALE1) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = 1;
  RCC_OscInitStruct.PLL.PLLN = 10;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV7;
  RCC_OscInitStruct.PLL.PLLQ = RCC_PLLQ_DIV2;
  RCC_OscInitStruct.PLL.PLLR = RCC_PLLR_DIV2;
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
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_4) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief TIM16 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM16_Init(void)
{

  /* USER CODE BEGIN TIM16_Init 0 */

  /* USER CODE END TIM16_Init 0 */

  /* USER CODE BEGIN TIM16_Init 1 */

  /* USER CODE END TIM16_Init 1 */
  htim16.Instance = TIM16;
  htim16.Init.Prescaler = 0;
  htim16.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim16.Init.Period = 65535;
  htim16.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim16.Init.RepetitionCounter = 0;
  htim16.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim16) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM16_Init 2 */

  /* USER CODE END TIM16_Init 2 */

}

/**
  * @brief USART1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART1_UART_Init(void)
{

  /* USER CODE BEGIN USART1_Init 0 */

  /* USER CODE END USART1_Init 0 */

  /* USER CODE BEGIN USART1_Init 1 */

  /* USER CODE END USART1_Init 1 */
  huart1.Instance = USART1;
  huart1.Init.BaudRate = 9600;
  huart1.Init.WordLength = UART_WORDLENGTH_8B;
  huart1.Init.StopBits = UART_STOPBITS_1;
  huart1.Init.Parity = UART_PARITY_NONE;
  huart1.Init.Mode = UART_MODE_TX_RX;
  huart1.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart1.Init.OverSampling = UART_OVERSAMPLING_16;
  huart1.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  huart1.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
  if (HAL_UART_Init(&huart1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART1_Init 2 */

  /* USER CODE END USART1_Init 2 */

}

/**
  * @brief USART2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART2_UART_Init(void)
{

  /* USER CODE BEGIN USART2_Init 0 */

  /* USER CODE END USART2_Init 0 */

  /* USER CODE BEGIN USART2_Init 1 */

  /* USER CODE END USART2_Init 1 */
  huart2.Instance = USART2;
  huart2.Init.BaudRate = 9600;
  huart2.Init.WordLength = UART_WORDLENGTH_8B;
  huart2.Init.StopBits = UART_STOPBITS_1;
  huart2.Init.Parity = UART_PARITY_NONE;
  huart2.Init.Mode = UART_MODE_TX_RX;
  huart2.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart2.Init.OverSampling = UART_OVERSAMPLING_16;
  huart2.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  huart2.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
  if (HAL_UART_Init(&huart2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART2_Init 2 */

  /* USER CODE END USART2_Init 2 */

}

/**
  * @brief USART3 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART3_UART_Init(void)
{

  /* USER CODE BEGIN USART3_Init 0 */

  /* USER CODE END USART3_Init 0 */

  /* USER CODE BEGIN USART3_Init 1 */

  /* USER CODE END USART3_Init 1 */
  huart3.Instance = USART3;
  huart3.Init.BaudRate = 115200;
  huart3.Init.WordLength = UART_WORDLENGTH_8B;
  huart3.Init.StopBits = UART_STOPBITS_1;
  huart3.Init.Parity = UART_PARITY_NONE;
  huart3.Init.Mode = UART_MODE_TX_RX;
  huart3.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart3.Init.OverSampling = UART_OVERSAMPLING_16;
  huart3.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  huart3.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
  if (HAL_UART_Init(&huart3) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART3_Init 2 */

  /* USER CODE END USART3_Init 2 */

}

/**
  * Enable DMA controller clock
  */
static void MX_DMA_Init(void)
{

  /* DMA controller clock enable */
  __HAL_RCC_DMA1_CLK_ENABLE();

  /* DMA interrupt init */
  /* DMA1_Channel5_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Channel5_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Channel5_IRQn);
  /* DMA1_Channel7_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Channel7_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Channel7_IRQn);

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};
/* USER CODE BEGIN MX_GPIO_Init_1 */
/* USER CODE END MX_GPIO_Init_1 */

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_5, GPIO_PIN_RESET);

  /*Configure GPIO pin : B1_Pin */
  GPIO_InitStruct.Pin = B1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(B1_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : PA5 */
  GPIO_InitStruct.Pin = GPIO_PIN_5;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

/* USER CODE BEGIN MX_GPIO_Init_2 */
/* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
//	char uart_buf[50];
//	int uart_buf_len;
//	uint16_t timer_val;

	if(huart->Instance==USART1)
	{
//  		timer_val = __HAL_TIM_GET_COUNTER(&htim16);

		//sprintf(debug_message, "Getting Location Data...\r\n");
		//HAL_UART_Transmit(&huart2, (uint8_t *)debug_message, DEBUG_MESSAGE_SIZE, 100);
		memcpy(gps_tx_buffer, gps_rx_buffer, GPS_RX_BUFFER_SIZE);
		// Extract data only if the GPRMC sentence is found
		if (strstr((char *)gps_tx_buffer, "$GPRMC") != NULL)
			{
				extractGPRMCData(gps_tx_buffer);
				// Format the Time, latitude and longitude into a message
				memset(location_data, 0, sizeof(location_data)); // clear the message buffer or else same data of the buffer will be tx'd again and again.
				memset(location_std_format, 0, sizeof(location_std_format)); // clear the message buffer or else same data of the buffer will be tx'd again and again.

				// Construct message with all information
				strncat((char *)location_data, "Time: ", sizeof(location_data) - strlen((char *)location_data) - 1);
				strncat((char *)location_data, time, sizeof(location_data) - strlen((char *)location_data) - 1);
				strncat((char *)location_data, "\n", sizeof(location_data) - strlen((char *)location_data) - 1); // Include \r with \n
				strncat((char *)location_data, "Latitude: ", sizeof(location_data) - strlen((char *)location_data) - 1);
				strncat((char *)location_data, latitude, sizeof(location_data) - strlen((char *)location_data) - 1);
				strncat((char *)location_data, "\n", sizeof(location_data) - strlen((char *)location_data) - 1); // Include \r with \n
				strncat((char *)location_data, "Latitude Direction: ", sizeof(location_data) - strlen((char *)location_data) - 1);
				strncat((char *)location_data, latitudeDirection, sizeof(location_data) - strlen((char *)location_data) - 1);
				strncat((char *)location_data, "\n", sizeof(location_data) - strlen((char *)location_data) - 1); // Include \r with \n
				strncat((char *)location_data, "Longitude: ", sizeof(location_data) - strlen((char *)location_data) - 1);
				strncat((char *)location_data, longitude, sizeof(location_data) - strlen((char *)location_data) - 1);
				strncat((char *)location_data, "\n", sizeof(location_data) - strlen((char *)location_data) - 1); // Include \r with \n
				strncat((char *)location_data, "Longitude Direction: ", sizeof(location_data) - strlen((char *)location_data) - 1);
				strncat((char *)location_data, longitudeDirection, sizeof(location_data) - strlen((char *)location_data) - 1);
				strncat((char *)location_data, "\n", sizeof(location_data) - strlen((char *)location_data) - 1); // Include \r with \n
				strncat((char *)location_data, "Location: ", sizeof(location_data) - strlen((char *)location_data) - 1);
				strncat((char *)location_data, latitude, sizeof(location_data) - strlen((char *)location_data) - 1);
				strncat((char *)location_data, ",", sizeof(location_data) - strlen((char *)location_data) - 1); // Include , between lat. and long.
				strncat((char *)location_data, longitude, sizeof(location_data) - strlen((char *)location_data) - 1);
				// Create location in standard format
				strncat((char *)location_std_format, "Location: ", sizeof(location_std_format) - strlen((char *)location_std_format) - 1);
				strncat((char *)location_std_format, latitude, sizeof(location_std_format) - strlen((char *)location_std_format) - 1);
				strncat((char *)location_std_format, ",", sizeof(location_std_format) - strlen((char *)location_std_format) - 1); // Include , between lat. and long.
				strncat((char *)location_std_format, longitude, sizeof(location_std_format) - strlen((char *)location_std_format) - 1);
				// Transmit the formatted message using DMA

			    if (strlen((char *)latitude) != '\0' || strlen((char *)longitude) != '\0')
			    	{
			    	  //HAL_UART_Transmit_DMA(&huart2, location_std_format, strlen((char *)location_std_format));
//					  sprintf(debug_message, "Time taken to get Gps Fix\r\n");
//					  HAL_UART_Transmit(&huart2, (uint8_t *)debug_message, DEBUG_MESSAGE_SIZE, 100);
//					  timer_val = __HAL_TIM_GET_COUNTER(&htim16) - timer_val;
//					  uart_buf_len = sprintf(uart_buf, "%u us \r\n", timer_val);
//					  HAL_UART_Transmit(&huart2, (uint8_t *)uart_buf, uart_buf_len, 100);

			    	}
			    else
			    	{
			    		memset(location_data, 0, sizeof(location_data)); // clear the location_data buffer or else same data of the buffer will be tx'd again and again.
			    		// Construct location_data with all information
			    		sprintf(debug_message, "Invalid Format Location data\r\n");
			    		HAL_UART_Transmit(&huart2, (uint8_t *)debug_message, DEBUG_MESSAGE_SIZE, 100);
			    	}

			}
		else
			{
				sprintf(debug_message, "Finding GPS fix...\r\n");
				HAL_UART_Transmit_DMA(&huart2, (uint8_t *)debug_message, DEBUG_MESSAGE_SIZE);
			}
			//Continuously receive from UART.
			HAL_UART_Receive_DMA(&huart1, gps_rx_buffer , GPS_RX_BUFFER_SIZE);
	}
}


// Function to extract data from the $GPRMC sentence
void extractGPRMCData(uint8_t *buffer) {
    char *sentenceStart = strstr((char *)buffer, "$GPRMC");

    if (sentenceStart != NULL) {
        char *token;

        // Extract the time field (1st field after $GPRMC)
        token = strtok(sentenceStart, ",");
        token = strtok(NULL, ",");
        if (token != NULL) {
            strncpy(time, token, sizeof(time) - 1);
            time[sizeof(time) - 1] = '\0';

            // Convert time from UTC to PST
            int hours = (time[0] - '0') * 10 + (time[1] - '0');
            int minutes = (time[2] - '0') * 10 + (time[3] - '0');
            int seconds = (time[4] - '0') * 10 + (time[5] - '0');
            hours -= 8; // Convert UTC to PST
            if (hours < 0) {
                hours += 24;
            }
            snprintf(time, 15, "%02d:%02d:%02d", hours, minutes, seconds);
        }

        // Extract the status field
        token = strtok(NULL, ",");
        if (token != NULL) {
            status = token[0];
        }

        // Extract and convert latitude and longitude if status is 'A'
        if (status == 'A') {
            // Latitude
            token = strtok(NULL, ",");
            if (token != NULL) {
                strncpy(latitude, token, sizeof(latitude) - 1);
                latitude[sizeof(latitude) - 1] = '\0';

                // Parse latitude
                int degrees = (latitude[0] - '0') * 10 + (latitude[1] - '0');
                int minutes = (latitude[2] - '0') * 10 + (latitude[3] - '0');
                int fraction = 0;
                int fractionLen = 0;
                for (int i = 5; latitude[i] != '\0' && latitude[i] != ','; ++i) {
                    fraction = fraction * 10 + (latitude[i] - '0');
                    fractionLen++;
                }

                // Compute scaling factor manually (10^fractionLen)
                int scalingFactor = 1;
                for (int i = 0; i < fractionLen; i++) {
                    scalingFactor *= 10;
                }

                int seconds = (fraction * 60) / scalingFactor; // Scale to seconds

                token = strtok(NULL, ",");//Latitude Direction
                if (token != NULL && token[0] == 'S') {
                	snprintf(latitude, 20, "-%d*%02d.%02d'", degrees, minutes, seconds);
                	strncpy(latitudeDirection, token, sizeof(latitudeDirection) - 1);
                }
                else{
                    snprintf(latitude, 20, "%d*%02d.%02d'", degrees, minutes, seconds);
                    strncpy(latitudeDirection, token, sizeof(latitudeDirection) - 1);
                }
            }

//            // Latitude Direction
//            token = strtok(NULL, ",");
//            if (token != NULL) {
//                strncpy(latitudeDirection, token, sizeof(latitudeDirection) - 1);
//            }

            // Longitude
            token = strtok(NULL, ",");
            if (token != NULL) {
                strncpy(longitude, token, sizeof(longitude) - 1);
                longitude[sizeof(longitude) - 1] = '\0';

                // Parse longitude
                int degrees = (longitude[0] - '0') * 100 + (longitude[1] - '0') * 10 + (longitude[2] - '0');
                int minutes = (longitude[3] - '0') * 10 + (longitude[4] - '0');
                int fraction = 0;
                int fractionLen = 0;
                for (int i = 6; longitude[i] != '\0' && longitude[i] != ','; ++i) {
                    fraction = fraction * 10 + (longitude[i] - '0');
                    fractionLen++;
                }

                // Compute scaling factor manually (10^fractionLen)
                int scalingFactor = 1;
                for (int i = 0; i < fractionLen; i++) {
                    scalingFactor *= 10;
                }

                int seconds = (fraction * 60) / scalingFactor; // Scale to seconds

                token = strtok(NULL, ","); // Longitude direction (E/W)
                if (token != NULL && token[0] == 'W') {
                	snprintf(longitude, 20, "-%d*%02d.%02d'", degrees, minutes, seconds);
                	strncpy(longitudeDirection, token, sizeof(longitudeDirection) - 1);
                }
                else{
                    snprintf(longitude, 20, "%d*%02d.%02d'", degrees, minutes, seconds);
                    strncpy(longitudeDirection, token, sizeof(longitudeDirection) - 1);
                }
            }
        }
    }
}




int HAL_UART_ReceiveAvailable(UART_HandleTypeDef *huart)
{
    // Check if the UART's receive data register is not empty
    if (__HAL_UART_GET_FLAG(huart, UART_FLAG_RXNE)) {
        // Clear the RXNE flag (optional as it is cleared when reading data)
        return 1;  // Data available to read
    }
    return 0;  // No data available
}

int Extract_SMS_Number(uint8_t *buffer)
{
    // Convert buffer to null-terminated string
    const char *data_str = (const char *)buffer;
    // Locate the "+CMTI:" string in the buffer
    const char *start = strstr(data_str, "+CMTI:");
    if (start == NULL) {
        return -1; // "+CMTI:" not found
    }
    // Locate the "SM" part within "+CMTI:"
    start = strstr(start, "\"SM\",");
    if (start == NULL) {
        return -1; // "\"SM\"," not found
    }
    // Move past "\"SM\","
    start += strlen("\"SM\",");
    // Extract the number after "\"SM\","
    int sms_number = -1;
    if (sscanf(start, "%d", &sms_number) == 1) {
        // Validate the extracted number is within the valid range (0-29)
        if (sms_number >= 0 && sms_number <= 29) {
            return sms_number; // Valid SMS number
        }
    }
    return -1; // Return error if parsing fails or number is out of range
}

bool isArrayZero(const uint8_t *array, size_t size) {
    for (size_t i = 0; i < size; i++) {
        if (array[i] != 0) {
            return false;  // Found a non-zero element
        }
    }
    return true;  // All elements are zero
}

int Extract_SMS_Message(uint8_t *sms_data, char *output_message, size_t output_size)
{
    // Convert the uint8_t buffer to a null-terminated string
    char *data_str = (char *)sms_data;
    // Locate the first instance of "\r\n" to skip the header
    char *start = strstr(data_str, "\r\n");
    if (start == NULL) {
        // If "\r\n" not found, return failure
        return 0;
    }

    // Move to the start of the actual message (skip "\r\n")
    start += 2;
    // Locate the next "\r\n" which marks the end of the message
    char *end = strstr(start, "\r\n");
    if (end == NULL) {
        // If the end "\r\n" not found, return failure
        return 0;
    }

    end += 2;
    char *final_end = strstr(end, "\r\n");
        if (final_end == NULL) {
            // If the end "\r\n" not found, return failure
            return 0;
        }
    // Calculate the message length
    size_t message_length = final_end - end;
    if (message_length >= output_size) {
        // Truncate if message exceeds the output buffer size
        message_length = output_size - 1;
    }
    // Copy the message content to the output buffer and null-terminate it
    strncpy(output_message, end, message_length);
    output_message[message_length] = '\0';
    return 1;  // Return success
}

void SendSMS(char *mobileNumber, char *message) {
    sprintf(ATcommand, "AT+CMGS=\"%s\"\r\n", mobileNumber);
    HAL_UART_Transmit(&huart3, (uint8_t *)ATcommand, strlen(ATcommand), 100);
    HAL_Delay(100);
    HAL_UART_Transmit(&huart3, (uint8_t *)message, strlen(message), 100);
    sprintf(ATcommand, "%c\r\n", 0x1a); // Ctrl+Z
    HAL_UART_Transmit(&huart3, (uint8_t *)ATcommand, strlen(ATcommand), 100);

}

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
