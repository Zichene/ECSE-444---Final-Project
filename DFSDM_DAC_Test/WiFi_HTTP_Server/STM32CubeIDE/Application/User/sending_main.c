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
#include "sending_board_config.h"

#ifdef SENDING_ACTIVE
/* Private defines -----------------------------------------------------------*/
#define PORT           10 // PORT of this board [sending board]
#define REMOTE_PORT    80 // PORT Of the receiving board
#define REMOTE_SOCKET	0 // SOCKET of the receiving board
#define TERMINAL_USE

#define WIFI_WRITE_TIMEOUT 10000
#define WIFI_READ_TIMEOUT  10000
#define SOCKET                 1


#ifdef  TERMINAL_USE
#define LOG(a) printf a
#else
#define LOG(a)
#endif


/* Private typedef------------------------------------------------------------*/
/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
#if defined (TERMINAL_USE)
extern UART_HandleTypeDef hDiscoUart;
#endif /* TERMINAL_USE */

static  uint8_t http[1024];
static  uint8_t  IP_Addr[4];
static uint8_t RemoteIP_Addr[4]; // address of receiving board

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
static int wifi_send_data_to_board(char* data);




/* Private functions ---------------------------------------------------------*/
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

  /*Initialize Temperature sensor */
 // HAL_ADCEx_Calibration_Start(&AdcHandle, ADC_SINGLE_ENDED);
  //HAL_ADC_Start(&AdcHandle) ;

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
  BSP_TSENSOR_Init();

  printf("****** SENDING BOARD Initiating ****** \r\n");

#endif /* TERMINAL_USE */
    while(wifi_connect_to_board(REMOTE_IP_0, REMOTE_IP_1, REMOTE_IP_2, REMOTE_IP_3) != WIFI_STATUS_OK) {
    	printf("Retrying to connect to board ... \r\n");
    }
    // successfully connected to board
    printf("Closed client connection \r\n");
    wifi_send_data_to_board("mega balls");
    wifi_send_data_to_board("hfsdhfjds");
    wifi_send_data_to_board("kkkkk");
    wifi_send_data_to_board("232242");

}

/**
 * This function attempts to sent an HTTP request to the other board, which contains some data.
 */
static int wifi_send_data_to_board(char* data) {
	// first try to connect to the board
    if (WIFI_OpenClientConnection(REMOTE_SOCKET, WIFI_TCP_PROTOCOL, "test", RemoteIP_Addr, REMOTE_PORT, PORT) != WIFI_STATUS_OK) {
    	printf("Could not connect to other board \r\n");
    	return -1;
    }
	char* http_header = "HTTP/1.0 200 OK\r\nContent-Type: text/plain\r\n";
    strcpy((char *)http, http_header);
    strcat((char *)http, data);
    uint16_t actualSent;
    /*
    if (WIFI_SendDataTo(REMOTE_SOCKET, http, sizeof(http), &actualSent, WIFI_WRITE_TIMEOUT, RemoteIP_Addr, REMOTE_PORT)!= WIFI_STATUS_OK) {
    	printf("ERROR: Could not send data, check remote socket & remote IP \r\n");
        memset(http, 0, strlen(http_header)+strlen(data));
    	return -1;
    } */
    if (WIFI_SendData(REMOTE_SOCKET, http, sizeof(http),&actualSent, WIFI_WRITE_TIMEOUT) != WIFI_STATUS_OK) {
    	printf("Could not send data \r\n");
        memset(http, 0, strlen(http));
        return -1;
    }
    printf("Sent out %d bytes\r\n", actualSent);
    memset(http, 0, strlen(http));
    // wait for resp from receiving board before proceeding
    uint8_t resp[1000];
    memset(resp, 0, strlen((char*)resp));
    uint16_t actualReceived;
    /*
    while(WIFI_ReceiveDataFrom(REMOTE_SOCKET, resp, 1000, &actualReceived, WIFI_READ_TIMEOUT, RemoteIP_Addr, 7, REMOTE_PORT) != WIFI_STATUS_OK) {

    }
    */
    while(WIFI_ReceiveData(REMOTE_SOCKET, resp, 1000, &actualReceived, WIFI_READ_TIMEOUT) != WIFI_STATUS_OK) {

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
    default:
    {
      break;
    }
  }
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
#endif


