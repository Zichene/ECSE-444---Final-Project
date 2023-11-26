
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include <math.h>
#include "receiving_board_config.h"
#include "arm_math.h"
#ifdef RECEIVING_ACTIVE
#ifdef __ICCARM__
#include <LowLevelIOInterface.h>
#endif

/* Private defines -----------------------------------------------------------*/
#define PORT           80
#define TERMINAL_USE
#define WIFI_WRITE_TIMEOUT 10000
#define WIFI_READ_TIMEOUT  10000
#define SOCKET                 0
#ifdef  TERMINAL_USE
#define LOG(a) printf a
#else
#define LOG(a)
#endif
#define SSID_SIZE     100
#define PASSWORD_SIZE 100
#define N_FFT 2048
#define BUFFER_LENGTH 20*N_FFT
#define SENDING_LENGTH 1000
//Default sampling rate is at 10kHz.
#define SAMPLING_RATE_FACTOR_DAC 10000
#define CLOCK_DIVIDER_DFSDM 100

/* Private typedef------------------------------------------------------------*/
/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
DAC_HandleTypeDef hdac1;
DMA_HandleTypeDef hdma_dac1_ch1;
DFSDM_Filter_HandleTypeDef hdfsdm1_filter0;
DFSDM_Channel_HandleTypeDef hdfsdm1_channel2;
DMA_HandleTypeDef hdma_dfsdm1_flt0;
TIM_HandleTypeDef htim2;
arm_rfft_fast_instance_f32 S_RFFT_F_I;
#if defined (TERMINAL_USE)
extern UART_HandleTypeDef hDiscoUart;
#endif /* TERMINAL_USE */

static  uint8_t http[1024];
static  uint8_t  IP_Addr[4];
uint8_t resp[SENDING_LENGTH];
uint32_t play[BUFFER_LENGTH];
int buffer_index = 0;

/** INTERRUPT FLAGS **/
volatile uint8_t DFSDM_finished = false; // flag

/* Private function prototypes -----------------------------------------------*/
#if defined (TERMINAL_USE)
#ifdef __GNUC__
/* With GCC, small printf (option LD Linker->Libraries->Small printf
   set to 'Yes') calls __io_putchar() */
#define PUTCHAR_PROTOTYPE int __io_putchar(int ch)
#define GETCHAR_PROTOTYPE int __io_getchar(void)
#else
#define PUTCHAR_PROTOTYPE int fputc(int ch, FILE *f)
#define GETCHAR_PROTOTYPE int fgetc(FILE *f)
#endif /* __GNUC__ */
#endif /* TERMINAL_USE */

static void SystemClock_Config(void);
static WIFI_Status_t wifi_process_received_data(void);
static int send_receive_confirmation(void);
static int wifi_wait_data_from_board(void);
static int wifi_start(void);
static void receiveFFTBlock(uint16_t blockIndex, uint16_t respLen);
static void Button_ISR(void);
/* MX Inits */
static void MX_DMA_Init(void);
static void MX_DAC1_Init(void);
static void MX_TIM2_Init(void);
static void MX_DFSDM1_Init(void);


/* Private functions ---------------------------------------------------------*/
void transformBufferToDAC(int32_t *buffer, uint32_t recording_buffer_length);
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

	/* USER push button is used to ask if reconfiguration is needed */
	BSP_PB_Init(BUTTON_USER, BUTTON_MODE_EXTI);

	/* Initialize all configured peripherals */
	MX_DMA_Init();
	MX_DAC1_Init();
	MX_TIM2_Init();
	MX_DFSDM1_Init();

	/* initiating things */
	HAL_DAC_Start(&hdac1, DAC_CHANNEL_1);
	// start the timer (TIM2) and associated interrupt
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

	printf("****** RECEIVING BOARD Initiating ******\r\n");

#endif /* TERMINAL_USE */

	// wifi_server();
	wifi_wait_data_from_board();

	while (1){

	}
}

static int wifi_wait_data_from_board(void) {
	// start the wifi module
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
			LOG(("ERROR : es-wifi module CANNOT get IP address\r\n"));
			return -1;
		}
	}
	else
	{
		LOG(("ERROR : es-wifi module NOT connected\r\n"));
		return -1;
	}

	// starting server
	if (WIFI_STATUS_OK != WIFI_StartServer(SOCKET, WIFI_TCP_PROTOCOL, 1, "", PORT)) {
		LOG(("ERROR: Could not start server \r\n"));
		return -1;
	}
	uint8_t RemoteIP[4];
	uint16_t RemotePort;

	while(true) {
		// wait for connection: wait for either a post req. or a get req. from client
		printf("Waiting for client connection \r\n");
		while (WIFI_STATUS_OK != WIFI_WaitServerConnection(SOCKET, 1000, RemoteIP, sizeof(RemoteIP), &RemotePort))
		{
			LOG(("."));
		}
		LOG(("\nClient connected %d.%d.%d.%d:%d\r\n",RemoteIP[0],RemoteIP[1],RemoteIP[2],RemoteIP[3],RemotePort));

		// process server
		printf("Processing server \r\n");
		wifi_process_received_data();

		// close connection
		printf("Closing current connection \r\n");
		if (WIFI_STATUS_OK != WIFI_CloseServerConnection(SOCKET)) {
			LOG(("Server could not be closed \r\n"));
			return -1;
		}

	}
}


static WIFI_Status_t wifi_process_received_data() {
	// get the resp
	WIFI_Status_t ret;
	uint16_t respLen;
	if (WIFI_STATUS_OK == WIFI_ReceiveData(SOCKET, resp, SENDING_LENGTH, &respLen, WIFI_READ_TIMEOUT)) {
		if (respLen > 0) {
			// send_receive_confirmation(); maybe we dont need to send back a confirmation for now
			printf(" \r\nReceived audio");
			// need to use uint32_t array
			/************** CONVERT FFT RECEIVED INTO PROPER uint32_t **************/
			receiveFFTBlock(buffer_index, respLen);
			/*
			for(int i = 0; i < SENDING_LENGTH; i++){
				play[buffer_index * SENDING_LENGTH + i] = (uint32_t)resp[i];
			}
			*/
			printf("\r\nReceived Packet #%d\r\n", buffer_index + 1);
			buffer_index++;

			if(buffer_index == BUFFER_LENGTH/N_FFT){
				buffer_index = 0;
				HAL_DAC_Start_DMA(&hdac1, DAC_CHANNEL_1, play, BUFFER_LENGTH, DAC_ALIGN_8B_R);
			}

			send_receive_confirmation();
		}
		else {
			ret = WIFI_STATUS_ERROR;
		}
	}
	return WIFI_STATUS_OK;
}

static int send_receive_confirmation() {
	uint16_t SentDataLength;
	WIFI_Status_t ret;
	char* http_header = "HTTP/1.0 200 OK\r\nContent-Type: text/plain\r\nPragma: no-cache\r\n\r\n";
	char* message = "Message received\r\n";
	strcpy((char *)http, http_header);
	strcat((char *)http, message);
	ret = WIFI_SendData(0, (uint8_t *)http, strlen(http), &SentDataLength, WIFI_WRITE_TIMEOUT);
	if (ret != WIFI_STATUS_OK && (SentDataLength != strlen(http))) {
		ret = WIFI_STATUS_ERROR;
	}
	printf("Request sent out to sending board\r\n: %s", http);
	memset(http, 0, strlen(http)); // clear the http var after usage
	return ret;
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
		printf("eS-WiFi Initialized.\r\n");
		if(WIFI_GetMAC_Address(MAC_Addr, sizeof(MAC_Addr)) == WIFI_STATUS_OK)
		{
			LOG(("eS-WiFi module MAC Address : %02X:%02X:%02X:%02X:%02X:%02X\r\n",
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
void transformBufferToDAC(int32_t *buffer, uint32_t recording_buffer_length) {
	for (int i = 0; i < recording_buffer_length; i++) {
		int32_t val = buffer[i]; // 24-bit value
		val = val >> 8; // remove this for LOUDER but MORE SCUFFED NOISE
		// need to map buffer values to 8bit right alligned values (uint8_t)
		// from experimentation (screaming at the board): min values tend to be -3000 and max seems to be ~1000
		const int16_t MAX_VAL = 2000;
		const int16_t MIN_VAL = -1500;
		const float a = (255.0)/(MAX_VAL - MIN_VAL); // slope

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
			buffer[i] = val; // change the buffer
		} else {
			Error_Handler(); // should not happen
		}
	}
}

/**
 * Once a block of audio FFT data has been received (received in q7_t format) in [resp]
 * Convert the q7_t values in resp to float32_t using arm_q7_to_float.
 * Apply inverse FFT
 * Rescale values to be in [0, 255] range and int32_t
 * Store these values in the right place given the blockIndex in the play buffer
 */
static void receiveFFTBlock(uint16_t blockIndex, uint16_t respLen) {
	/* STEP 1: Perform Inverse FFT */
	float ifftInputBuf[N_FFT];
	float ifftOutputBuf[N_FFT];
	memset(ifftInputBuf, 0, N_FFT*sizeof(float)); //SETS everything to 0 since resp has a SMALLER length then ifftInputBuf
	arm_q7_to_float(resp, ifftInputBuf, respLen);
	/* Now ifftInputBuf should contain the values in resp along with a long string of zeroes  */
	if (arm_rfft_fast_init_f32(&S_RFFT_F_I, N_FFT) != ARM_MATH_SUCCESS) {
		printf("ERROR: Couldn't initialize FFT instance \r\n");
		return;
	}
	/* The last argument to indicate that we want an inverse FFT */
	arm_rfft_fast_f32(&S_RFFT_F_I, ifftInputBuf, ifftOutputBuf, true);

	/* STEP 2: Convert into [0, 255]*/
	/* Find maximum and minimum elements of ifftOutputBuf */
	float min_f = MAXFLOAT;
	float max_f = -MAXFLOAT;
	for (int i = 0; i < N_FFT; i++) {
		float cur = ifftOutputBuf[i];
		if (cur < min_f)
			cur = min_f;
		if (cur > max_f)
			cur = max_f;
	}
	float scalingFactor;
	if ((max_f - min_f)!= 0) {
		scalingFactor = (255)/(max_f-min_f);
	} else {
		printf("ERROR: Unexpected division by zero in receiveFFTBlock \r\n");
		return;
	}
	uint16_t offset = blockIndex*N_FFT;
	for (int i = 0; i < N_FFT; i++) {
		/*[minimum, maximum]  ->  [0, maximum-minimum]*/
		float cur = ifftOutputBuf[i];
		cur = cur - min_f;
		/*[0, maximum-minimum] -> [0, 255]*/
		cur = cur * scalingFactor;
		play[offset+i] = (int32_t) roundf(cur);
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
	hdfsdm1_channel2.Init.OutputClock.Divider = CLOCK_DIVIDER_DFSDM;
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
	htim2.Init.Period = SAMPLING_RATE_FACTOR_DAC;
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


#ifdef __ICCARM__
/**
 * @brief
 * @param
 * @retval
 */
size_t __read(int handle, unsigned char * buffer, size_t size)
{
	int nChars = 0;

	/* handle ? */

	for (/* Empty */; size > 0; --size)
	{
		uint8_t ch = 0;
		while (HAL_OK != HAL_UART_Receive(&hDiscoUart, (uint8_t *)&ch, 1, 30000))
		{
			;
		}

		*buffer++ = ch;
		++nChars;
	}

	return nChars;
}
#elif defined(__CC_ARM) || defined(__GNUC__)
/**
 * @brief  Retargets the C library scanf function to the USART.
 * @param  None
 * @retval None
 */
GETCHAR_PROTOTYPE
{
	/* Place your implementation of fgetc here */
	/* e.g. read a character on USART and loop until the end of read */
	uint8_t ch = 0;
	while (HAL_OK != HAL_UART_Receive(&hDiscoUart, (uint8_t *)&ch, 1, 40000))
	{
		;
	}
	return ch;
}
#endif /* defined(__CC_ARM)  */
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
	case (USER_BUTTON_PIN):
    				{
		Button_ISR();
		break;
    				}
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

void HAL_DFSDM_FilterRegConvCpltCallback(DFSDM_Filter_HandleTypeDef *hdfsdm_filter) {
	DFSDM_finished = true;
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
 * @brief Update button ISR status
 */
static void Button_ISR(void)
{

}

void HAL_TIM_PeriodElapsedCallback (TIM_HandleTypeDef *htim) {
	//int test;
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
