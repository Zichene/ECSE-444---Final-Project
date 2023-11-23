/**
  ******************************************************************************
  * @file    Wifi/WiFi_HTTP_Server/src/main.c
  * @author  MCD Application Team
  * @brief   This file provides main program functions
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2017 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
//#define SENDING_ACTIVE // Comment/Uncomment this depending on if you are this board as sending/receiving
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include <math.h>
#include "sending_board_config.h"
#define ARM_MATH_CM4
#include "arm_math.h"

#ifdef SENDING_ACTIVE
/* Private defines -----------------------------------------------------------*/
#define PORT           10 // PORT of this board [sending board]
#define REMOTE_PORT    80 // PORT Of the receiving board
#define REMOTE_SOCKET	0 // SOCKET of the receiving board
#define TERMINAL_USE

#define WIFI_WRITE_TIMEOUT 10000
#define WIFI_READ_TIMEOUT  10000
#define SOCKET                 1
#define SENDING_BUFLEN 1000
#define RECORDING_BUFLEN 40000
#define SAMPLING_RATE_FACTOR 2 // DEFAULT SAMPLING IS AT 20 kHz, a higher factor means a lower sampling rate


#ifdef  TERMINAL_USE
#define LOG(a) printf a
#else
#define LOG(a)
#endif

/* Private typedef------------------------------------------------------------*/
/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
DAC_HandleTypeDef hdac1;
DMA_HandleTypeDef hdma_dac1_ch1;
DFSDM_Filter_HandleTypeDef hdfsdm1_filter0;
DFSDM_Channel_HandleTypeDef hdfsdm1_channel2;
DMA_HandleTypeDef hdma_dfsdm1_flt0;
TIM_HandleTypeDef htim2;
#if defined (TERMINAL_USE)
extern UART_HandleTypeDef hDiscoUart;
#endif /* TERMINAL_USE */

//static  uint8_t http[1024];
static  uint8_t  IP_Addr[4];
static uint8_t RemoteIP_Addr[4]; // address of receiving board
static int32_t recordingBuffer[RECORDING_BUFLEN]; // sampling rate @ 20 kHz => 2 second of recording and 1 second of space for chime
static uint8_t sendingBuffer[RECORDING_BUFLEN]; // sampling rate @ 20 kHz => 2 second of recording and 1 second of space for chime

/** INTERRUPT FLAGS **/
volatile uint8_t DFSDM_half_finished=false;
volatile uint8_t DFSDM_finished=false;
volatile uint8_t buttonPressed=0;

/* Private function prototypes -----------------------------------------------*/
#if defined (TERMINAL_USE)
#ifdef __GNUC__
/* With GCC, small printf (option LD Linker->Libraries->Small printf
   set to 'Yes') calls __io_putchar() */
#define PUTCHAR_PROTOTYPE int __io_putchar(int ch)
#else
#define PUTCHAR_PROTOTYPE int fputc(int ch, FILE *f)
#endif /* __GNUC__ */
#endif /* TERMINAL_USE */

static void SystemClock_Config(void);
static int wifi_start(void);
static int wifi_connect_to_board(uint8_t ip0, uint8_t ip1, uint8_t ip2, uint8_t ip3);
static int wifi_send_data_to_board(uint8_t* data);

/* MX Inits */
static void MX_DMA_Init(void);
static void MX_DAC1_Init(void);
static void MX_TIM2_Init(void);
static void MX_DFSDM1_Init(void);
static void MX_GPIO_Init(void);

/* Private functions ---------------------------------------------------------*/
void transformBufferToDAC(int32_t *buffer, uint32_t recording_buffer_length, uint8_t *outputBuffer);
/**
  * @brief  Main program
  * @param  None
  * @retval None
  */
int main(void)
{
  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* Configure the system clock */
  SystemClock_Config();

  /* Configure LED2 */
  BSP_LED_Init(LED2);

  /* Initialize all configured peripherals */
  MX_DMA_Init();
  MX_DAC1_Init();
  MX_TIM2_Init();
  MX_DFSDM1_Init();
  MX_GPIO_Init();


  //HAL_DAC_Start(&hdac1, DAC_CHANNEL_1);
  HAL_TIM_Base_Start_IT(&htim2); // the _IT at the end of fn. means interrupt

  /* WIFI Web Server demonstration */
#if defined (TERMINAL_USE)
  /* Initialize all configured peripherals */
  hDiscoUart.Instance = DISCOVERY_COM1;
  hDiscoUart.Init.BaudRate = 115200;
  hDiscoUart.Init.WordLength = UART_WORDLENGTH_8B;
  hDiscoUart.Init.StopBits = UART_STOPBITS_1;
  hDiscoUart.Init.Parity = UART_PARITY_NONE;
  hDiscoUart.Init.Mode = UART_MODE_TX_RX;
  hDiscoUart.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  hDiscoUart.Init.OverSampling = UART_OVERSAMPLING_16;
  hDiscoUart.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  hDiscoUart.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;


  BSP_COM_Init(COM1, &hDiscoUart);

  /* resetting buffers */
  memset(recordingBuffer, 0, RECORDING_BUFLEN*sizeof(int32_t));
  memset(sendingBuffer, 0, SENDING_BUFLEN);

  printf("****** SENDING BOARD Initiating ****** \r\n");

#endif /* TERMINAL_USE */


	while(wifi_connect_to_board(REMOTE_IP_0, REMOTE_IP_1, REMOTE_IP_2, REMOTE_IP_3) != WIFI_STATUS_OK) {
		printf("Retrying to connect to board ... \r\n");
	}
	// successfully connected to board
	printf("Connected to other board. \r\n");
	//wifi_send_data_to_board("test");

    while (true) {
    printf("Waiting for button press to send mic data.\r\n");
    while(buttonPressed==false){
    	__WFI(); // Waiting for interrupt mode?
    }
    buttonPressed = false;
   // while (buttonPressed == false) {
    	// IF YOU WANT TO USE THE HALF_FINISHED THING U HAVE TO SET DMA TO CIRCULAR IN hal_msp.c
    	if (HAL_DFSDM_FilterRegularStart_DMA(&hdfsdm1_filter0, recordingBuffer, RECORDING_BUFLEN) != HAL_OK) {
    	  printf("Failed to start DFSDM\r\n");
    	}
    	/*
		while (!DFSDM_half_finished) {
		}
		// need to transform only first half
		printf("DFSDM half finished, sending first half of buffer");
		DFSDM_half_finished = false;
		transformBufferToDAC(recordingBuffer, VOICE_BUFLEN/2, sendingBuffer);
		HAL_DAC_Start_DMA(&hdac1, DAC_CHANNEL_1, recordingBuffer, VOICE_BUFLEN/2, DAC_ALIGN_8B_R);
		//wifi_send_data_to_board(sendingBuffer);
		 */
		while (!DFSDM_finished) {
		}

		DFSDM_finished = false;
		HAL_DFSDM_FilterRegularStop_DMA(&hdfsdm1_filter0); //  this is necessary
		/* Need to transform only the last half*/
		/*
		transformBufferToDAC(&(recordingBuffer[VOICE_BUFLEN/2]), VOICE_BUFLEN/2, sendingBuffer);
		HAL_DAC_Start_DMA(&hdac1, DAC_CHANNEL_1, &(recordingBuffer[VOICE_BUFLEN/2]), VOICE_BUFLEN/2, DAC_ALIGN_8B_R);
		wifi_send_data_to_board(sendingBuffer);
		*/
		/* test playback on this board */
		transformBufferToDAC(recordingBuffer,RECORDING_BUFLEN, sendingBuffer);
		if (HAL_DAC_Start_DMA(&hdac1, DAC_CHANNEL_1, recordingBuffer, RECORDING_BUFLEN, DAC_ALIGN_8B_R) != HAL_OK) {
		  printf("Failed to start DAC");
		}

		printf("Sending audio to other board \r\n");

		uint16_t step = RECORDING_BUFLEN/SENDING_BUFLEN;
		for (int i = 0 ; i < step; i++) {
			if(wifi_send_data_to_board(&(sendingBuffer[i*SENDING_BUFLEN])) == WIFI_STATUS_OK) {
				printf("Sent packet %d\r\n", i);
			}
		}
   // }
    buttonPressed = false;
  }
}

/**
 * This function attempts to sent an HTTP request to the other board, which contains some data.
 */
static int wifi_send_data_to_board(uint8_t* data) {
	// first try to connect to the board

    if (WIFI_OpenClientConnection(REMOTE_SOCKET, WIFI_TCP_PROTOCOL, "test", RemoteIP_Addr, REMOTE_PORT, PORT) != WIFI_STATUS_OK) {
    	printf("Could not connect to other board \r\n");
    	return -1;
    }

    printf("Connected to other board\r\n");
	//char* http_header = "HTTP/1.0 200 OK\r\nContent-Type: text/plain\r\n";
    //strcat(http_header, data);

    uint16_t actualSent;
    /*
    for (int i = 0; i < 20; i++) {
    	if (WIFI_SendData(REMOTE_SOCKET, &(data[i*1000]), SENDING_BUFLEN, &actualSent, WIFI_WRITE_TIMEOUT) == WIFI_STATUS_OK) {
			printf("Sent packet %d \r\n", i);
			//memset(data, 0, strlen(data));
			return -1;
    	}
    }
    */
	if (WIFI_SendData(REMOTE_SOCKET,data, SENDING_BUFLEN, &actualSent, WIFI_WRITE_TIMEOUT) != WIFI_STATUS_OK) {
		printf("Could not send data \r\n");
		//memset(data, 0, strlen(data));
		return -1;
	}

    //memset(data, 0, strlen(data)); // maybe we can remove this
    // wait for resp from receiving board before proceeding
    uint8_t resp[100];
    memset(resp, 0, strlen((char*)resp));
    uint16_t actualReceived;

    while(WIFI_ReceiveData(REMOTE_SOCKET, resp, 100, &actualReceived, WIFI_READ_TIMEOUT) != WIFI_STATUS_OK) {

    }
    printf("Received response from receiving board: %s\r\n", resp);

    // close connection

    if (WIFI_CloseClientConnection(REMOTE_SOCKET) != WIFI_STATUS_OK) {
    	printf("Could not close client connection \r\n");
    	return -1;
    }
    printf("Closed client connection \r\n");

    return WIFI_STATUS_OK;
}
/**
 * This function takes care of:
 * - starting the es-wifi module
 * - connecting to a network whose credentials are provided in code
 * - connect to the receiving board using ip address provided in parameters of this function
 * - returns -1 if anything bad happens
 * - returns WIFI_STATUS_OK if successful
 */
static int wifi_connect_to_board(uint8_t ip0, uint8_t ip1, uint8_t ip2, uint8_t ip3) {
	// start wifi module
	wifi_start();


	// connect to existing AP
	if (WIFI_Connect(SSID, PASSWORD, SECURITY) == WIFI_STATUS_OK)
	{
	  if(WIFI_GetIP_Address(IP_Addr, sizeof(IP_Addr)) == WIFI_STATUS_OK)
	  {
		LOG(("eS-WiFi module connected: got IP Address : %d.%d.%d.%d\r\n",
				 IP_Addr[0],
				 IP_Addr[1],
				 IP_Addr[2],
				 IP_Addr[3]));
	  }
	  else
	  {
		LOG((" ERROR : es-wifi module CANNOT get IP address\r\n"));
		return -1;
	  }
	}
	else
	{
	   LOG(("ERROR : es-wifi module NOT connected\r\n"));
	   return -1;
	}
	// start server (?)

	if (WIFI_STATUS_OK != WIFI_StartServer(SOCKET, WIFI_TCP_PROTOCOL, 1, "", PORT)) {
		printf("ERROR: Could not start server \r\n");
		return -1;
	}

	// trying to connect to other board using IP addr
    RemoteIP_Addr[0] = ip0;
    RemoteIP_Addr[1] = ip1;
    RemoteIP_Addr[2] = ip2;
    RemoteIP_Addr[3] = ip3;
    if (WIFI_OpenClientConnection(REMOTE_SOCKET, WIFI_TCP_PROTOCOL, "test", RemoteIP_Addr, REMOTE_PORT, PORT) != WIFI_STATUS_OK) {
    	printf("Could not connect to other board \r\n");
    	return -1;
    }

    // we need to disconnect at the end since we will be reconnecting to send data
    // close connection
    if (WIFI_CloseClientConnection(REMOTE_SOCKET) != WIFI_STATUS_OK) {
    	printf("Could not close client connection \r\n");
    	return -1;
    }

    return WIFI_STATUS_OK;
}

/**
  * @brief  Send HTML page
  * @param  None
  * @retval None
  */


static int wifi_start(void)
{
  uint8_t  MAC_Addr[6];

 /*Initialize and use WIFI module */
  if(WIFI_Init() ==  WIFI_STATUS_OK)
  {
    LOG(("ES-WIFI Initialized.\r\n"));
    if(WIFI_GetMAC_Address(MAC_Addr, sizeof(MAC_Addr)) == WIFI_STATUS_OK)
    {
      LOG(("> eS-WiFi module MAC Address : %02X:%02X:%02X:%02X:%02X:%02X\r\n",
               MAC_Addr[0],
               MAC_Addr[1],
               MAC_Addr[2],
               MAC_Addr[3],
               MAC_Addr[4],
               MAC_Addr[5]));
    }
    else
    {
      LOG(("> ERROR : CANNOT get MAC address\r\n"));
      return -1;
    }
  }
  else
  {
    return -1;
  }
  return 0;
}

/**
 * Transforms a buffer's values into valid DAC 8bit right aligned values
 */
void transformBufferToDAC(int32_t *buffer, uint32_t recording_buffer_length, uint8_t *outputBuffer) {
	// need to map buffer values to 8bit right alligned values (uint8_t)
	// from experimentation (screaming at the board): min values tend to be -3000 and max seems to be ~1000
	const int16_t MAX_VAL = 1000;
	const int16_t MIN_VAL = -1000;
	const float a = (255.0)/(MAX_VAL - MIN_VAL); // slope
	for (int i = 0; i < recording_buffer_length; i++) {
		int32_t val = buffer[i]; // 24-bit value
		val = val >> 8; // remove this for LOUDER but MORE SCUFFED NOISE
		// clip buffer values to within [-MIN_VAL, MAX_VAL]
		if (val <= MIN_VAL) {
			val = MIN_VAL;
		}
		if (val >= MAX_VAL) {
			val = MAX_VAL;
		}
		// scale values up by [-MIN_VAL] to make sure no negatives
		if (MIN_VAL < 0) {
			val += (-MIN_VAL);
		}
		// now the range of val should be [0, MAX_VAL-MIN_VAL], apply linear function to get DAC val
		val = round(a*val);
		if (val >= 0 && val <= 255) {
			outputBuffer[i] = (uint8_t)val; // change the buffer
			buffer[i]=val;
		} else {
			Error_Handler(); // should not happen
		}
	}
}

/**
  * @brief  System Clock Configuration
  *         The system Clock is configured as follow :
  *            System Clock source            = PLL (MSI)
  *            SYSCLK(Hz)                     = 80000000
  *            HCLK(Hz)                       = 80000000
  *            AHB Prescaler                  = 1
  *            APB1 Prescaler                 = 1
  *            APB2 Prescaler                 = 1
  *            MSI Frequency(Hz)              = 4000000
  *            PLL_M                          = 1
  *            PLL_N                          = 40
  *            PLL_R                          = 2
  *            PLL_P                          = 7
  *            PLL_Q                          = 4
  *            Flash Latency(WS)              = 4
  * @param  None
  * @retval None
  */
static void SystemClock_Config(void)
{
  RCC_ClkInitTypeDef RCC_ClkInitStruct;
  RCC_OscInitTypeDef RCC_OscInitStruct;

  /* MSI is enabled after System reset, activate PLL with MSI as source */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_MSI;
  RCC_OscInitStruct.MSIState = RCC_MSI_ON;
  RCC_OscInitStruct.MSIClockRange = RCC_MSIRANGE_6;
  RCC_OscInitStruct.MSICalibrationValue = RCC_MSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_MSI;
  RCC_OscInitStruct.PLL.PLLM = 1;
  RCC_OscInitStruct.PLL.PLLN = 40;
  RCC_OscInitStruct.PLL.PLLR = 2;
  RCC_OscInitStruct.PLL.PLLP = 7;
  RCC_OscInitStruct.PLL.PLLQ = 4;
  if(HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    /* Initialization Error */
    while(1);
  }

  /* Select PLL as system clock source and configure the HCLK, PCLK1 and PCLK2
     clocks dividers */
  RCC_ClkInitStruct.ClockType = (RCC_CLOCKTYPE_SYSCLK | RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2);
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;
  if(HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_4) != HAL_OK)
  {
    /* Initialization Error */
    while(1);
  }
}
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};
/* USER CODE BEGIN MX_GPIO_Init_1 */
/* USER CODE END MX_GPIO_Init_1 */

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOC_CLK_ENABLE();

  /*Configure GPIO pin : PC13 */
  GPIO_InitStruct.Pin = GPIO_PIN_13;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /* EXTI interrupt init*/
  HAL_NVIC_SetPriority(EXTI15_10_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI15_10_IRQn);

/* USER CODE BEGIN MX_GPIO_Init_2 */
/* USER CODE END MX_GPIO_Init_2 */
}

#if defined (TERMINAL_USE)
/**
  * @brief  Retargets the C library printf function to the USART.
  * @param  None
  * @retval None
  */
PUTCHAR_PROTOTYPE
{
  /* Place your implementation of fputc here */
  /* e.g. write a character to the USART1 and Loop until the end of transmission */
  HAL_UART_Transmit(&hDiscoUart, (uint8_t *)&ch, 1, 0xFFFF);

  return ch;
}

#endif /* TERMINAL_USE */

#ifdef USE_FULL_ASSERT

/**
   * @brief Reports the name of the source file and the source line number
   * where the assert_param error has occurred.
   * @param file: pointer to the source file name
   * @param line: assert_param error line source number
   * @retval None
   */
void assert_failed(uint8_t* file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
    ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */

}

#endif

static void MX_DAC1_Init(void)
{

  /* USER CODE BEGIN DAC1_Init 0 */

  /* USER CODE END DAC1_Init 0 */

  DAC_ChannelConfTypeDef sConfig = {0};

  /* USER CODE BEGIN DAC1_Init 1 */

  /* USER CODE END DAC1_Init 1 */

  /** DAC Initialization
  */
  hdac1.Instance = DAC1;
  if (HAL_DAC_Init(&hdac1) != HAL_OK)
  {
    Error_Handler();
  }

  /** DAC channel OUT1 config
  */
  sConfig.DAC_SampleAndHold = DAC_SAMPLEANDHOLD_DISABLE;
  sConfig.DAC_Trigger = DAC_TRIGGER_T2_TRGO;
  sConfig.DAC_HighFrequency = DAC_HIGH_FREQUENCY_INTERFACE_MODE_ABOVE_80MHZ;
  sConfig.DAC_OutputBuffer = DAC_OUTPUTBUFFER_ENABLE;
  sConfig.DAC_ConnectOnChipPeripheral = DAC_CHIPCONNECT_DISABLE;
  sConfig.DAC_UserTrimming = DAC_TRIMMING_FACTORY;
  if (HAL_DAC_ConfigChannel(&hdac1, &sConfig, DAC_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }

  /** DAC channel OUT2 config
  */
  sConfig.DAC_Trigger = DAC_TRIGGER_NONE;
  if (HAL_DAC_ConfigChannel(&hdac1, &sConfig, DAC_CHANNEL_2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN DAC1_Init 2 */

  /* USER CODE END DAC1_Init 2 */

}

/**
  * @brief DFSDM1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_DFSDM1_Init(void)
{

  /* USER CODE BEGIN DFSDM1_Init 0 */

  /* USER CODE END DFSDM1_Init 0 */

  /* USER CODE BEGIN DFSDM1_Init 1 */

  /* USER CODE END DFSDM1_Init 1 */
  hdfsdm1_filter0.Instance = DFSDM1_Filter0;
  hdfsdm1_filter0.Init.RegularParam.Trigger = DFSDM_FILTER_SW_TRIGGER;
  hdfsdm1_filter0.Init.RegularParam.FastMode = ENABLE;
  hdfsdm1_filter0.Init.RegularParam.DmaMode = ENABLE;
  hdfsdm1_filter0.Init.FilterParam.SincOrder = DFSDM_FILTER_FASTSINC_ORDER;
  hdfsdm1_filter0.Init.FilterParam.Oversampling = 100;
  hdfsdm1_filter0.Init.FilterParam.IntOversampling = 1;
  if (HAL_DFSDM_FilterInit(&hdfsdm1_filter0) != HAL_OK)
  {
    Error_Handler();
  }
  hdfsdm1_channel2.Instance = DFSDM1_Channel2;
  hdfsdm1_channel2.Init.OutputClock.Activation = ENABLE;
  hdfsdm1_channel2.Init.OutputClock.Selection = DFSDM_CHANNEL_OUTPUT_CLOCK_SYSTEM;
  hdfsdm1_channel2.Init.OutputClock.Divider = 50*SAMPLING_RATE_FACTOR; // CHANGED FROM 50 to 100 so that SAMPLING RATE IS 10K
  hdfsdm1_channel2.Init.Input.Multiplexer = DFSDM_CHANNEL_EXTERNAL_INPUTS;
  hdfsdm1_channel2.Init.Input.DataPacking = DFSDM_CHANNEL_STANDARD_MODE;
  hdfsdm1_channel2.Init.Input.Pins = DFSDM_CHANNEL_SAME_CHANNEL_PINS;
  hdfsdm1_channel2.Init.SerialInterface.Type = DFSDM_CHANNEL_SPI_RISING;
  hdfsdm1_channel2.Init.SerialInterface.SpiClock = DFSDM_CHANNEL_SPI_CLOCK_INTERNAL;
  hdfsdm1_channel2.Init.Awd.FilterOrder = DFSDM_CHANNEL_FASTSINC_ORDER;
  hdfsdm1_channel2.Init.Awd.Oversampling = 1;
  hdfsdm1_channel2.Init.Offset = 0;
  hdfsdm1_channel2.Init.RightBitShift = 0x00;
  if (HAL_DFSDM_ChannelInit(&hdfsdm1_channel2) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_DFSDM_FilterConfigRegChannel(&hdfsdm1_filter0, DFSDM_CHANNEL_2, DFSDM_CONTINUOUS_CONV_ON) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN DFSDM1_Init 2 */

  /* USER CODE END DFSDM1_Init 2 */

}

/**
  * @brief TIM2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM2_Init(void)
{

  /* USER CODE BEGIN TIM2_Init 0 */

  /* USER CODE END TIM2_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM2_Init 1 */

  /* USER CODE END TIM2_Init 1 */
  htim2.Instance = TIM2;
  htim2.Init.Prescaler = 0;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = 5000*SAMPLING_RATE_FACTOR;
  htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim2) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim2, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_UPDATE;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim2, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM2_Init 2 */

  /* USER CODE END TIM2_Init 2 */

}

/**
  * Enable DMA controller clock
  */
static void MX_DMA_Init(void)
{

  /* DMA controller clock enable */
  __HAL_RCC_DMAMUX1_CLK_ENABLE();
  __HAL_RCC_DMA1_CLK_ENABLE();

  /* DMA interrupt init */
  /* DMA1_Channel1_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Channel1_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Channel1_IRQn);
  /* DMA1_Channel2_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Channel2_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Channel2_IRQn);

}

/**
  * @brief  EXTI line detection callback.
  * @param  GPIO_Pin: Specifies the port pin connected to corresponding EXTI line.
  * @retval None
  */

void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
  switch (GPIO_Pin)
  {
  case (GPIO_PIN_1):
    {
      SPI_WIFI_ISR();
      break;
    }
  case (GPIO_PIN_13):
     {
      buttonPressed=1;
        break;
     }
    default:
    {
      break;
    }
  }
}
/*
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin){
	if(GPIO_Pin==GPIO_PIN_1){
	    SPI_WIFI_ISR();
	}
	else if (GPIO_Pin==USER_BUTTON_PIN)
    {
     buttonPressed=true;
    }
}*/

void HAL_DFSDM_FilterRegConvCpltCallback(DFSDM_Filter_HandleTypeDef *hdfsdm_filter) {
	DFSDM_finished = true;
}

void HAL_DFSDM_FilterRegConvHalfCpltCallback (DFSDM_Filter_HandleTypeDef * hdfsdm_filter) {
	DFSDM_half_finished = true;
}
/**
  * @brief  SPI3 line detection callback.
  * @param  None
  * @retval None
  */
void SPI3_IRQHandler(void)
{
  HAL_SPI_IRQHandler(&hspi);
}

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
#endif



