/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2025 STMicroelectronics.
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
//#include "Wire.h"
#include "RF24.h"
#include "RF24_mesh.h"
#include "aht20.h"
#include "bmp180.h"

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define NO_SLEEP
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
ADC_HandleTypeDef hadc;

I2C_HandleTypeDef hi2c1;

UART_HandleTypeDef hlpuart1;

RTC_HandleTypeDef hrtc;

SPI_HandleTypeDef hspi1;

/* USER CODE BEGIN PV */
uint8_t wait = 0;
volatile uint32_t vcc = 0;
volatile uint8_t connected = 0;

uint8_t buf[32];
char uart_buf[64];
uint8_t len;

typedef struct {
  int8_t temp;
  uint8_t humidity;
  uint16_t pressure;
  uint16_t light;
  uint32_t timestamp;
} sensor_data_t;

sensor_data_t s_data;

#if 0
uint8_t frame_buffer[32];
uint8_t frame_size = 32;

uint64_t pipe = 0xF0F0F0F0D2LL;

static uint16_t next_id = 0;

typedef struct
{
    uint16_t from_node;
    uint16_t to_node;
    uint16_t id;
    unsigned char type;
    unsigned char reserved;
} RF24NetworkHeader;
#endif

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_RTC_Init(void);
static void MX_ADC_Init(void);
static void MX_LPUART1_UART_Init(void);
static void MX_SPI1_Init(void);
static void MX_I2C1_Init(void);
/* USER CODE BEGIN PFP */
static void SystemPower_Config(void);
uint16_t board_GetVcc(void);
uint16_t board_GetLight(void);
uint32_t board_ConvertToUnixTime(RTC_DateTypeDef *psDate, RTC_TimeTypeDef *psTime);

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
void USART_TX (uint8_t* dt, uint16_t sz)
{
  uint16_t ind = 0;
  while (ind<sz)
  {
    while (!LL_LPUART_IsActiveFlag_TXE(LPUART1)) {}
    LL_LPUART_TransmitData8(LPUART1,*(uint8_t*)(dt+ind));
    ind++;
  }
}
#if 0
uint64_t pipe_address(uint16_t node, uint8_t pipe)
{

    static uint8_t address_translation[] = { 0xc3,
                                             0x3c,
                                             0x33,
                                             0xce,
                                             0x3e,
                                             0xe3,
                                             0xec
    };
    uint64_t result = 0xCCCCCCCCCCLL;
    uint8_t* out = (uint8_t*)(&result);

    // Translate the address to use our optimally chosen radio address bytes
    uint8_t count = 1;
    uint16_t dec = node;

    while (dec) {

        if (pipe != 0 || !node)

            out[count] = address_translation[(dec % 8)]; // Convert our decimal values to octal, translate them to address bytes, and set our address

        dec /= 8;
        count++;
    }


    if (pipe != 0 || !node)
        out[0] = address_translation[pipe];
    else
        out[1] = address_translation[count - 1];

    return result;
}

uint16_t node_address = 0x0924;

uint8_t _multicast_level = 0;
uint8_t parent_pipe = 0;
uint16_t node_mask = 0;
uint16_t parent_node;

void setup_address(void)
{
    // First, establish the node_mask
    uint16_t node_mask_check = 0xFFFF;

    uint8_t count = 0;


    while (node_address & node_mask_check) {
        node_mask_check <<= 3;

        count++;
    }
    _multicast_level = count;


    node_mask = ~node_mask_check;

    // parent mask is the next level down
    uint16_t parent_mask = node_mask >> 3;

    // parent node is the part IN the mask
    parent_node = node_address & parent_mask;

    // parent pipe is the part OUT of the mask
    uint16_t i = node_address;
    uint16_t m = parent_mask;
    while (m) {
        i >>= 3;
        m >>= 3;
    }
    parent_pipe = i;
}


uint8_t write_to_pipe(uint16_t node, uint8_t pipe, uint8_t multicast)
{
    uint8_t ok = false;

    stopListening();

    setAutoAckPipe(0, !multicast);
    openWritingPipe(pipe_address(node, pipe));

    ok = write(frame_buffer, frame_size);

    setAutoAckPipe(0, 0);
    /*
    ok = writeFast(frame_buffer, frame_size, 0);

    ok = txStandBy(85);
    setAutoAckPipe(0, 0);
*/
//    if (!(networkFlags & FLAG_FAST_FRAG)) {
//        ok = txStandBy(txTimeout);
//                setAutoAck(0, 0);
//    }
//    else if (!ok) {
//        ok = txStandBy(txTimeout);
//    }

    return ok;
}
#endif

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{
  /* USER CODE BEGIN 1 */
	RTC_TimeTypeDef sTime = {0};
	RTC_DateTypeDef sDate = {0};
	RTC_AlarmTypeDef sAlarm = {0};
	ADC_ChannelConfTypeDef sAdcConf = {0};
	//RF24NetworkHeader *hdr;
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
  MX_RTC_Init();
  MX_ADC_Init();
  //MX_LPUART1_UART_Init();
  MX_SPI1_Init();
  MX_I2C1_Init();
  /* USER CODE BEGIN 2 */
  LL_GPIO_SetOutputPin(GPIOA, LIGHT_EN_Pin);

  mesh_Init();

  mesh_AddressRequest();

  connected = mesh_Lookup();



#if 0
  NRF_Init();


  stopListening();

  setChannel(97);
  //mesh addr 0x0924
  setAutoAck(1);
  setAutoAckPipe(0, 0);
  enableDynamicPayloads();


  //begin(node_address)
  // Use different retry periods to reduce data collisions
  uint8_t retryVar = (((node_address % 6) + 1) * 2) + 3;
  setRetries(retryVar, 5); // max about 85ms per attempt
  //txTimeout = 25;
  //routeTimeout = txTimeout * 3; // Adjust for max delay per node within a single chain

  // Setup our address helper cache
  setup_address();

    // Open up all listening pipes
    uint8_t i = 6;
    while (i--)
      openReadingPipe(i, pipe_address(node_address, i));

   startListening();

  //openWritingPipe(pipe);

   // net poll
   hdr = (RF24NetworkHeader *)frame_buffer;

    hdr->from_node = 0x0924;
    hdr->to_node = 0x0040;
    hdr->id = next_id++;
    hdr->type = 0xC2;
    hdr->reserved = 0;

    frame_size = 8;

    write_to_pipe(0, 0, 1);

    startListening();

    while(availableMy() == 0);

    frame_size = getDynamicPayloadSize();
    read(frame_buffer, frame_size);

    HAL_Delay(10);


    // address request
    hdr = (RF24NetworkHeader *)frame_buffer;

    hdr->from_node = 0x0924;
    hdr->to_node = 0x0000;
    hdr->id = next_id++;
    hdr->type = 0xC3;
    hdr->reserved = 0x01; // node address

    frame_size = 8;

    write_to_pipe(0, 0, 1);

    startListening();

    while(availableMy() == 0);

    frame_size = getDynamicPayloadSize();
    read(frame_buffer, frame_size);

    memcpy(&node_address, &frame_buffer[8], 2);

    //begin(node_address)
    // Use different retry periods to reduce data collisions
    retryVar = (((node_address % 6) + 1) * 2) + 3;
    setRetries(retryVar, 5); // max about 85ms per attempt
    //txTimeout = 25;
    //routeTimeout = txTimeout * 3; // Adjust for max delay per node within a single chain

    // Setup our address helper cache
    setup_address();

    // Open up all listening pipes
    i = 6;
    while (i--)
      openReadingPipe(i, pipe_address(node_address, i));

    startListening();


    HAL_Delay(10);

    //mesh lookup
    hdr = (RF24NetworkHeader *)frame_buffer;

    hdr->from_node = node_address;
    hdr->to_node = 0x0000;
    hdr->id = next_id++;
    hdr->type = 0xC6;
    hdr->reserved = 0x00; // node address

    memcpy(&frame_buffer[8], &node_address, 2);

    frame_size = 10;

    write_to_pipe(0, 5, 0);

    startListening();

    while(availableMy() == 0);

    frame_size = getDynamicPayloadSize();
    read(frame_buffer, frame_size);
#endif

  // нужна задержка, чтобы можно было прошивку записать/ прочитать
  // в режимах сна SWIO отключается
  LL_GPIO_SetOutputPin(GPIOA, GPIO_PIN_11);
  wait = 1;
  while(wait);
  LL_GPIO_ResetOutputPin(GPIOA, GPIO_PIN_11);

  // хрен знает поччему, но после повторной инициализции передача идет нормально
  mesh_Init();

  mesh_AddressRequest();
  HAL_Delay(200);
  connected = mesh_Lookup();

//  HAL_Delay(1000);
//  len = sprintf(uart_buf, "ready");
//  HAL_UART_Transmit(&hlpuart1, uart_buf, len, 2000);


  aht20_Init(&hi2c1);

  aht20_Measure();

  bmp180_Init(&hi2c1);
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
#ifndef NO_SLEEP
	  // все по новой
	  SystemClock_Config();

	  MX_GPIO_Init();
	  MX_ADC_Init();
	  //MX_LPUART1_UART_Init();
	  MX_SPI1_Init();
	  MX_I2C1_Init();
	  LL_GPIO_SetOutputPin(GPIOA, LIGHT_EN_Pin);
#endif
	  //HAL_GPIO_WritePin(GPIOA, LIGHT_EN_Pin, GPIO_PIN_SET);
//
//	  buf[3] = ~buf[0];
//		if(buf[3] != buf[1])
//		{
//		    LL_GPIO_SetOutputPin(GPIOA, GPIO_PIN_11);
//            HAL_Delay(200);
//            LL_GPIO_ResetOutputPin(GPIOA, GPIO_PIN_11);
//            HAL_Delay(200);
//            LL_GPIO_SetOutputPin(GPIOA, GPIO_PIN_11);
//            HAL_Delay(200);
//            LL_GPIO_ResetOutputPin(GPIOA, GPIO_PIN_11);
//            HAL_Delay(200);
//		}
//
//	  LL_GPIO_SetOutputPin(GPIOA, GPIO_PIN_11);
//	  HAL_Delay(200);
//	  LL_GPIO_ResetOutputPin(GPIOA, GPIO_PIN_11);
//	  HAL_Delay(10);

//	  LL_GPIO_SetOutputPin(GPIOA, VCC_EN_Pin);
//	  HAL_Delay(1);
//	  LL_GPIO_SetOutputPin(GPIOA, GPIO_PIN_11);
//	  sAdcConf.Channel = 0;
//	  sAdcConf.Rank = ADC_RANK_NONE;
//
//	  HAL_ADC_ConfigChannel(&hadc, &sAdcConf);
//	  HAL_ADC_Start(&hadc);
//	  HAL_ADC_PollForConversion(&hadc, 200);
//	  vcc = HAL_ADC_GetValue(&hadc);
//	  vcc = vcc*1412/1000;
//	  // r1*(r1+r2) = 0.492
//	  LL_GPIO_ResetOutputPin(GPIOA, GPIO_PIN_11);
//
//	  LL_GPIO_ResetOutputPin(GPIOA, VCC_EN_Pin);

	  powerUp();

	  vcc = board_GetVcc();

	  s_data.light = board_GetLight();

//	  buf[0] = vcc & 0xFF;
//	  buf[1] = ~buf[0];
//	  buf[2] = buf[0] + buf[1];

//	  len = sprintf(uart_buf, "vcc %u\r\n", vcc);
//	  HAL_UART_Transmit(&hlpuart1, uart_buf, len, 2000);
	  //USART_TX(uart_buf, len);

	  aht20_Measure();
	  s_data.temp = (int8_t )(aht20_GetTemp()/10);
	  s_data.humidity = (uint8_t )(aht20_GetHumidity()/10);

	  bmp180_GetTemp();
	  s_data.pressure = (uint16_t )(bmp180_GetPressure()/10);

	  s_data.timestamp = HAL_GetTick();
//	  len = sprintf(uart_buf, "temp %d C\r\n", aht20_GetTemp());
//	  HAL_UART_Transmit(&hlpuart1, uart_buf, len, 2000);
	  //USART_TX(uart_buf, len);

//      len = sprintf(uart_buf, "humidity %d %%\r\n\r\n", aht20_GetHumidity());
//      HAL_UART_Transmit(&hlpuart1, uart_buf, len, 2000);
      //USART_TX(uart_buf, len);

	  //HAL_SPI_Transmit(&hspi1, buf, 3, 2000);

	  //check connection

	  // запуск будильника на следующее пробуждение
	  HAL_RTC_DeactivateAlarm(&hrtc, RTC_ALARM_A);

	  //нужен имеено такй порядок чтения
	  HAL_RTC_GetTime(&hrtc, &sTime, RTC_FORMAT_BIN);
	  HAL_RTC_GetDate(&hrtc, &sDate, RTC_FORMAT_BIN);

	  s_data.timestamp = board_ConvertToUnixTime(&sDate, &sTime);

	  if(connected)
	  {
        // пробуем отправить сообщение
        connected = mesh_Write('T', &s_data, sizeof(sensor_data_t));
	  }
	  else
	  {
	      connected = mesh_AddressRequest();
	      // mesh_Lookup();
	  }

        sAlarm.AlarmTime.Hours = 0;//sTime.Hours;
        sAlarm.AlarmTime.Minutes = 0;//sTime.Minutes;
        sAlarm.AlarmTime.Seconds = (sTime.Seconds + 10 ) % 60;
        sAlarm.AlarmTime.SubSeconds = 0;
        sAlarm.AlarmTime.DayLightSaving = RTC_DAYLIGHTSAVING_NONE;
        sAlarm.AlarmTime.StoreOperation = RTC_STOREOPERATION_RESET;
        sAlarm.AlarmMask = RTC_ALARMMASK_DATEWEEKDAY|RTC_ALARMMASK_HOURS
                                    |RTC_ALARMMASK_MINUTES;
        sAlarm.AlarmSubSecondMask = RTC_ALARMSUBSECONDMASK_ALL;
        sAlarm.AlarmDateWeekDaySel = RTC_ALARMDATEWEEKDAYSEL_DATE;
        sAlarm.AlarmDateWeekDay = 1;
        sAlarm.Alarm = RTC_ALARM_A;
        if (HAL_RTC_SetAlarm_IT(&hrtc, &sAlarm, RTC_FORMAT_BIN) != HAL_OK)
        {
          Error_Handler();
        }

#ifdef NO_SLEEP
        powerDown();

		wait = 1;
		while(wait);
#else
		powerDown();

		/* Configure the system Power */
		SystemPower_Config();

		HAL_SuspendTick();

		/* Enter Stop Mode */
		HAL_PWR_EnterSTOPMode(PWR_LOWPOWERREGULATOR_ON, PWR_STOPENTRY_WFI);

		HAL_ResumeTick();
#endif
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
  LL_FLASH_SetLatency(LL_FLASH_LATENCY_0);
  while(LL_FLASH_GetLatency()!= LL_FLASH_LATENCY_0)
  {
  }
  LL_PWR_SetRegulVoltageScaling(LL_PWR_REGU_VOLTAGE_SCALE1);
  LL_RCC_HSI_Enable();

   /* Wait till HSI is ready */
  while(LL_RCC_HSI_IsReady() != 1)
  {

  }
  LL_RCC_HSI_SetCalibTrimming(16);
  LL_PWR_EnableBkUpAccess();
  if(LL_RCC_GetRTCClockSource() != LL_RCC_RTC_CLKSOURCE_LSE)
  {
    LL_RCC_ForceBackupDomainReset();
    LL_RCC_ReleaseBackupDomainReset();
  }
  LL_RCC_LSE_SetDriveCapability(LL_RCC_LSEDRIVE_LOW);
  LL_RCC_LSE_Enable();

   /* Wait till LSE is ready */
  while(LL_RCC_LSE_IsReady() != 1)
  {

  }
  if(LL_RCC_GetRTCClockSource() != LL_RCC_RTC_CLKSOURCE_LSE)
  {
    LL_RCC_SetRTCClockSource(LL_RCC_RTC_CLKSOURCE_LSE);
  }
  LL_RCC_EnableRTC();
  LL_RCC_SetAHBPrescaler(LL_RCC_SYSCLK_DIV_1);
  LL_RCC_SetAPB1Prescaler(LL_RCC_APB1_DIV_1);
  LL_RCC_SetAPB2Prescaler(LL_RCC_APB2_DIV_1);
  LL_RCC_SetSysClkSource(LL_RCC_SYS_CLKSOURCE_HSI);

   /* Wait till System clock is ready */
  while(LL_RCC_GetSysClkSource() != LL_RCC_SYS_CLKSOURCE_STATUS_HSI)
  {

  }
  LL_SetSystemCoreClock(16000000);

   /* Update the time base */
  if (HAL_InitTick (TICK_INT_PRIORITY) != HAL_OK)
  {
    Error_Handler();
  }
  LL_RCC_SetLPUARTClockSource(LL_RCC_LPUART1_CLKSOURCE_LSE);
  LL_RCC_SetI2CClockSource(LL_RCC_I2C1_CLKSOURCE_PCLK1);
}

/**
  * @brief ADC Initialization Function
  * @param None
  * @retval None
  */
static void MX_ADC_Init(void)
{

  /* USER CODE BEGIN ADC_Init 0 */

  /* USER CODE END ADC_Init 0 */

  ADC_ChannelConfTypeDef sConfig = {0};

  /* USER CODE BEGIN ADC_Init 1 */

  /* USER CODE END ADC_Init 1 */

  /** Configure the global features of the ADC (Clock, Resolution, Data Alignment and number of conversion)
  */
  hadc.Instance = ADC1;
  hadc.Init.OversamplingMode = DISABLE;
  hadc.Init.ClockPrescaler = ADC_CLOCK_ASYNC_DIV8;
  hadc.Init.Resolution = ADC_RESOLUTION_12B;
  hadc.Init.SamplingTime = ADC_SAMPLETIME_79CYCLES_5;
  hadc.Init.ScanConvMode = ADC_SCAN_DIRECTION_FORWARD;
  hadc.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc.Init.ContinuousConvMode = DISABLE;
  hadc.Init.DiscontinuousConvMode = DISABLE;
  hadc.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE;
  hadc.Init.ExternalTrigConv = ADC_SOFTWARE_START;
  hadc.Init.DMAContinuousRequests = DISABLE;
  hadc.Init.EOCSelection = ADC_EOC_SINGLE_CONV;
  hadc.Init.Overrun = ADC_OVR_DATA_PRESERVED;
  hadc.Init.LowPowerAutoWait = DISABLE;
  hadc.Init.LowPowerFrequencyMode = ENABLE;
  hadc.Init.LowPowerAutoPowerOff = DISABLE;
  if (HAL_ADC_Init(&hadc) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure for the selected ADC regular channel to be converted.
  */
  sConfig.Channel = ADC_CHANNEL_0;
  sConfig.Rank = ADC_RANK_CHANNEL_NUMBER;
  if (HAL_ADC_ConfigChannel(&hadc, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure for the selected ADC regular channel to be converted.
  */
  sConfig.Channel = ADC_CHANNEL_9;
  if (HAL_ADC_ConfigChannel(&hadc, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN ADC_Init 2 */

  /* USER CODE END ADC_Init 2 */

}

/**
  * @brief I2C1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_I2C1_Init(void)
{

  /* USER CODE BEGIN I2C1_Init 0 */

  /* USER CODE END I2C1_Init 0 */

  /* USER CODE BEGIN I2C1_Init 1 */

  /* USER CODE END I2C1_Init 1 */
  hi2c1.Instance = I2C1;
  hi2c1.Init.Timing = 0x00303D5B;
  hi2c1.Init.OwnAddress1 = 0;
  hi2c1.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c1.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c1.Init.OwnAddress2 = 0;
  hi2c1.Init.OwnAddress2Masks = I2C_OA2_NOMASK;
  hi2c1.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c1.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c1) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Analogue filter
  */
  if (HAL_I2CEx_ConfigAnalogFilter(&hi2c1, I2C_ANALOGFILTER_ENABLE) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Digital filter
  */
  if (HAL_I2CEx_ConfigDigitalFilter(&hi2c1, 0) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN I2C1_Init 2 */

  /* USER CODE END I2C1_Init 2 */

}

/**
  * @brief LPUART1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_LPUART1_UART_Init(void)
{

  /* USER CODE BEGIN LPUART1_Init 0 */

  /* USER CODE END LPUART1_Init 0 */

  /* USER CODE BEGIN LPUART1_Init 1 */

  /* USER CODE END LPUART1_Init 1 */
  hlpuart1.Instance = LPUART1;
  hlpuart1.Init.BaudRate = 9600;
  hlpuart1.Init.WordLength = UART_WORDLENGTH_8B;
  hlpuart1.Init.StopBits = UART_STOPBITS_1;
  hlpuart1.Init.Parity = UART_PARITY_NONE;
  hlpuart1.Init.Mode = UART_MODE_TX;
  hlpuart1.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  hlpuart1.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  hlpuart1.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
  if (HAL_UART_Init(&hlpuart1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN LPUART1_Init 2 */

  /* USER CODE END LPUART1_Init 2 */

}

/**
  * @brief RTC Initialization Function
  * @param None
  * @retval None
  */
static void MX_RTC_Init(void)
{

  /* USER CODE BEGIN RTC_Init 0 */

  /* USER CODE END RTC_Init 0 */

  RTC_TimeTypeDef sTime = {0};
  RTC_DateTypeDef sDate = {0};
  RTC_AlarmTypeDef sAlarm = {0};

  /* USER CODE BEGIN RTC_Init 1 */

  /* USER CODE END RTC_Init 1 */

  /** Initialize RTC Only
  */
  hrtc.Instance = RTC;
  hrtc.Init.HourFormat = RTC_HOURFORMAT_24;
  hrtc.Init.AsynchPrediv = 127;
  hrtc.Init.SynchPrediv = 255;
  hrtc.Init.OutPut = RTC_OUTPUT_DISABLE;
  hrtc.Init.OutPutRemap = RTC_OUTPUT_REMAP_NONE;
  hrtc.Init.OutPutPolarity = RTC_OUTPUT_POLARITY_HIGH;
  hrtc.Init.OutPutType = RTC_OUTPUT_TYPE_OPENDRAIN;
  if (HAL_RTC_Init(&hrtc) != HAL_OK)
  {
    Error_Handler();
  }

  /* USER CODE BEGIN Check_RTC_BKUP */
  HAL_RTC_GetTime(&hrtc, &sTime, RTC_FORMAT_BIN);
  HAL_RTC_GetDate(&hrtc, &sDate, RTC_FORMAT_BIN);

  if( (sDate.Year == 0) && (sDate.Month == 1) && (sDate.Date == 1) )
  {
        sTime.Hours = 12;
        sTime.Minutes = 15;
        sTime.Seconds = 0;
        sTime.DayLightSaving = RTC_DAYLIGHTSAVING_NONE;
        sTime.StoreOperation = RTC_STOREOPERATION_RESET;
        if (HAL_RTC_SetTime(&hrtc, &sTime, RTC_FORMAT_BIN) != HAL_OK)
        {
            Error_Handler();
        }

        sDate.WeekDay = RTC_WEEKDAY_SUNDAY;
        sDate.Month = RTC_MONTH_FEBRUARY;
        sDate.Date = 2;
        sDate.Year = 25;

        if (HAL_RTC_SetDate(&hrtc, &sDate, RTC_FORMAT_BIN) != HAL_OK)
        {
            Error_Handler();
        }
  }
  /* USER CODE END Check_RTC_BKUP */
#if 0
  /** Initialize RTC and set the Time and Date
  */
  sTime.Hours = 12;
  sTime.Minutes = 15;
  sTime.Seconds = 0;
  sTime.DayLightSaving = RTC_DAYLIGHTSAVING_NONE;
  sTime.StoreOperation = RTC_STOREOPERATION_RESET;
  if (HAL_RTC_SetTime(&hrtc, &sTime, RTC_FORMAT_BIN) != HAL_OK)
  {
    Error_Handler();
  }
  sDate.WeekDay = RTC_WEEKDAY_SUNDAY;
  sDate.Month = RTC_MONTH_JANUARY;
  sDate.Date = 12;
  sDate.Year = 25;

  if (HAL_RTC_SetDate(&hrtc, &sDate, RTC_FORMAT_BIN) != HAL_OK)
  {
    Error_Handler();
  }
#endif
  /** Enable the Alarm A
  */
  sAlarm.AlarmTime.Hours = sTime.Hours;//12;
  sAlarm.AlarmTime.Minutes = sTime.Minutes;//15;
  sAlarm.AlarmTime.Seconds = sTime.Seconds + 5;
  sAlarm.AlarmTime.SubSeconds = 0;
  sAlarm.AlarmTime.DayLightSaving = RTC_DAYLIGHTSAVING_NONE;
  sAlarm.AlarmTime.StoreOperation = RTC_STOREOPERATION_RESET;
  sAlarm.AlarmMask = RTC_ALARMMASK_DATEWEEKDAY|RTC_ALARMMASK_HOURS
                              |RTC_ALARMMASK_MINUTES;
  sAlarm.AlarmSubSecondMask = RTC_ALARMSUBSECONDMASK_ALL;
  sAlarm.AlarmDateWeekDaySel = RTC_ALARMDATEWEEKDAYSEL_DATE;
  sAlarm.AlarmDateWeekDay = 1;
  sAlarm.Alarm = RTC_ALARM_A;
  if (HAL_RTC_SetAlarm_IT(&hrtc, &sAlarm, RTC_FORMAT_BIN) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN RTC_Init 2 */

  /* USER CODE END RTC_Init 2 */

}

/**
  * @brief SPI1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_SPI1_Init(void)
{

  /* USER CODE BEGIN SPI1_Init 0 */

  /* USER CODE END SPI1_Init 0 */

  /* USER CODE BEGIN SPI1_Init 1 */

  /* USER CODE END SPI1_Init 1 */
  /* SPI1 parameter configuration*/
  hspi1.Instance = SPI1;
  hspi1.Init.Mode = SPI_MODE_MASTER;
  hspi1.Init.Direction = SPI_DIRECTION_2LINES;
  hspi1.Init.DataSize = SPI_DATASIZE_8BIT;
  hspi1.Init.CLKPolarity = SPI_POLARITY_LOW;
  hspi1.Init.CLKPhase = SPI_PHASE_1EDGE;
  hspi1.Init.NSS = SPI_NSS_SOFT;
  hspi1.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_2;
  hspi1.Init.FirstBit = SPI_FIRSTBIT_MSB;
  hspi1.Init.TIMode = SPI_TIMODE_DISABLE;
  hspi1.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  hspi1.Init.CRCPolynomial = 7;
  if (HAL_SPI_Init(&hspi1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN SPI1_Init 2 */

  /* USER CODE END SPI1_Init 2 */

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  LL_GPIO_InitTypeDef GPIO_InitStruct = {0};

  /* GPIO Ports Clock Enable */
  LL_IOP_GRP1_EnableClock(LL_IOP_GRP1_PERIPH_GPIOC);
  LL_IOP_GRP1_EnableClock(LL_IOP_GRP1_PERIPH_GPIOA);
  LL_IOP_GRP1_EnableClock(LL_IOP_GRP1_PERIPH_GPIOB);

  /**/
  LL_GPIO_ResetOutputPin(VCC_EN_GPIO_Port, VCC_EN_Pin);

  /**/
  LL_GPIO_ResetOutputPin(LIGHT_EN_GPIO_Port, LIGHT_EN_Pin);

  /**/
  LL_GPIO_ResetOutputPin(LED_GPIO_Port, LED_Pin);

  /**/
  LL_GPIO_ResetOutputPin(GPIOB, LL_GPIO_PIN_6);

  /**/
  LL_GPIO_SetOutputPin(GPIOB, LL_GPIO_PIN_7);

  /**/
  GPIO_InitStruct.Pin = VCC_EN_Pin;
  GPIO_InitStruct.Mode = LL_GPIO_MODE_OUTPUT;
  GPIO_InitStruct.Speed = LL_GPIO_SPEED_FREQ_LOW;
  GPIO_InitStruct.OutputType = LL_GPIO_OUTPUT_PUSHPULL;
  GPIO_InitStruct.Pull = LL_GPIO_PULL_NO;
  LL_GPIO_Init(VCC_EN_GPIO_Port, &GPIO_InitStruct);

  /**/
  GPIO_InitStruct.Pin = LIGHT_EN_Pin;
  GPIO_InitStruct.Mode = LL_GPIO_MODE_OUTPUT;
  GPIO_InitStruct.Speed = LL_GPIO_SPEED_FREQ_LOW;
  GPIO_InitStruct.OutputType = LL_GPIO_OUTPUT_PUSHPULL;
  GPIO_InitStruct.Pull = LL_GPIO_PULL_NO;
  LL_GPIO_Init(LIGHT_EN_GPIO_Port, &GPIO_InitStruct);

  /**/
  GPIO_InitStruct.Pin = LED_Pin;
  GPIO_InitStruct.Mode = LL_GPIO_MODE_OUTPUT;
  GPIO_InitStruct.Speed = LL_GPIO_SPEED_FREQ_LOW;
  GPIO_InitStruct.OutputType = LL_GPIO_OUTPUT_PUSHPULL;
  GPIO_InitStruct.Pull = LL_GPIO_PULL_NO;
  LL_GPIO_Init(LED_GPIO_Port, &GPIO_InitStruct);

  /**/
  GPIO_InitStruct.Pin = LL_GPIO_PIN_15;
  GPIO_InitStruct.Mode = LL_GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = LL_GPIO_PULL_NO;
  LL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /**/
  GPIO_InitStruct.Pin = LL_GPIO_PIN_6;
  GPIO_InitStruct.Mode = LL_GPIO_MODE_OUTPUT;
  GPIO_InitStruct.Speed = LL_GPIO_SPEED_FREQ_LOW;
  GPIO_InitStruct.OutputType = LL_GPIO_OUTPUT_PUSHPULL;
  GPIO_InitStruct.Pull = LL_GPIO_PULL_NO;
  LL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /**/
  GPIO_InitStruct.Pin = LL_GPIO_PIN_7;
  GPIO_InitStruct.Mode = LL_GPIO_MODE_OUTPUT;
  GPIO_InitStruct.Speed = LL_GPIO_SPEED_FREQ_LOW;
  GPIO_InitStruct.OutputType = LL_GPIO_OUTPUT_PUSHPULL;
  GPIO_InitStruct.Pull = LL_GPIO_PULL_NO;
  LL_GPIO_Init(GPIOB, &GPIO_InitStruct);

}

/* USER CODE BEGIN 4 */
void HAL_RTC_AlarmAEventCallback(RTC_HandleTypeDef *hrtc)
{
	wait = 0;
}

uint16_t board_GetVcc(void)
{
    uint32_t temp;
    ADC_ChannelConfTypeDef sAdcConf = {0};

    LL_GPIO_SetOutputPin(GPIOA, VCC_EN_Pin);
    HAL_Delay(1);
    sAdcConf.Channel = 0;
    sAdcConf.Rank = ADC_RANK_NONE;

    HAL_ADC_ConfigChannel(&hadc, &sAdcConf);
    HAL_ADC_Start(&hadc);
    HAL_ADC_PollForConversion(&hadc, 200);
    temp = HAL_ADC_GetValue(&hadc);
    temp = temp*1553/1000;
    // r1*(r1+r2) = 0.492
    LL_GPIO_ResetOutputPin(GPIOA, VCC_EN_Pin);

    return (uint16_t )temp;
}

uint16_t board_GetLight(void)
{
    uint32_t temp;
    ADC_ChannelConfTypeDef sAdcConf = {0};

    LL_GPIO_ResetOutputPin(GPIOA, LIGHT_EN_Pin);
    HAL_Delay(1);
    sAdcConf.Channel = 9;
    sAdcConf.Rank = ADC_RANK_NONE;

    HAL_ADC_ConfigChannel(&hadc, &sAdcConf);
    HAL_ADC_Start(&hadc);
    HAL_ADC_PollForConversion(&hadc, 200);
    temp = HAL_ADC_GetValue(&hadc);

    LL_GPIO_SetOutputPin(GPIOA, LIGHT_EN_Pin);

    return (uint16_t )temp;
}

#define _SEC_IN_MINUTE 60L
#define _SEC_IN_HOUR 3600L
#define _SEC_IN_DAY 86400L

static const int DAYS_IN_MONTH[12] =
{31, 28, 31, 30, 31, 30, 31, 31, 30, 31, 30, 31};

#define _DAYS_IN_MONTH(x) ((x == 1) ? days_in_feb : DAYS_IN_MONTH[x])

static const int _DAYS_BEFORE_MONTH[12] =
{0, 31, 59, 90, 120, 151, 181, 212, 243, 273, 304, 334};

#define _ISLEAP(y) (((y) % 4) == 0 && (((y) % 100) != 0 || (((y)+1900) % 400) == 0))
#define _DAYS_IN_YEAR(year) (_ISLEAP(year) ? 366 : 365)

uint32_t board_ConvertToUnixTime(RTC_DateTypeDef *psDate, RTC_TimeTypeDef *psTime)
{
#if 0
    uint32_t y;
    uint32_t m;
    uint32_t d;
    uint32_t t;

    //Year
    y = psDate->Year;
    //Month of year
    m = psDate->Month;
    //Day of month
    d = psDate->Date;

    //January and February are counted as months 13 and 14 of the previous year
    if(m <= 2)
    {
       m += 12;
       y -= 1;
    }

    //Convert years to days
    t = (365 * y) + (y / 4) - (y / 100) + (y / 400);
    //Convert months to days
    t += (30 * m) + (3 * (m + 1) / 5) + d;
    //Unix time starts on January 1st, 1970
    t -= 719561;
    //Convert days to seconds
    t *= 86400;
    //Add hours, minutes and seconds
    t += (3600 * psTime->Hours) + (60 * psTime->Minutes) + psTime->Seconds;

    //Return Unix time
    return t;
#endif
    uint32_t tim = 0;
    long days = 0;
    int year, isdst=0;
//    __tzinfo_type *tz = __gettzinfo ();
//
//    /* validate structure */
//    validate_structure (tim_p);
    psDate->Month--;

    psDate->Year += 100;

    /* compute hours, minutes, seconds */
    tim += psTime->Seconds + (psTime->Minutes * _SEC_IN_MINUTE) + (psTime->Hours * _SEC_IN_HOUR);

    /* compute days in year */
    days += psDate->Date - 1;
    days += _DAYS_BEFORE_MONTH[psDate->Month];

    if (psDate->Month > 1 && _DAYS_IN_YEAR (psDate->Year) == 366)
        days++;

    /* compute day of the year */
    //tim_p->tm_yday = days;

//    if (tim_p->tm_year > 10000 || tim_p->tm_year < -10000)
//      return (time_t) -1;

    /* compute days in other years */
    if ((year = psDate->Year) > 70)
    {
      for (year = 70; year < psDate->Year; year++)
          days += _DAYS_IN_YEAR (year);
    }
    else if (year < 70)
    {
      for (year = 69; year > psDate->Year; year--)
          days -= _DAYS_IN_YEAR (year);

      days -= _DAYS_IN_YEAR (year);
    }

    /* compute total seconds */
    tim += (uint32_t )days * _SEC_IN_DAY;

    return tim;
}

/**
  * @brief  System Power Configuration
  *         The system Power is configured as follow :
  *            + Regulator in LP mode
  *            + VREFINT OFF, with fast wakeup enabled
  *            + HSI as SysClk after Wake Up
  *            + No IWDG
  *            + Automatic Wakeup using RTC clocked by LSI (after ~4s)
  * @param  None
  * @retval None
  */
static void SystemPower_Config(void)
{
  GPIO_InitTypeDef GPIO_InitStructure = {0};

  /* Enable Power Control clock */
  __HAL_RCC_PWR_CLK_ENABLE();

  /* Enable Ultra low power mode */
  HAL_PWREx_EnableUltraLowPower();

  /* Enable the fast wake up from Ultra low power mode */
  HAL_PWREx_EnableFastWakeUp();

  /* Enable GPIOs clock */
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOH_CLK_ENABLE();

  /* Configure all GPIO port pins in Analog Input mode (floating input trigger OFF) */
  /* Note: Debug using ST-Link is not possible during the execution of this   */
  /*       example because communication between ST-link and the device       */
  /*       under test is done through UART. All GPIO pins are disabled (set   */
  /*       to analog input mode) including  UART I/O pins.           */
  GPIO_InitStructure.Pin = GPIO_PIN_All;
  GPIO_InitStructure.Mode = GPIO_MODE_ANALOG;
  GPIO_InitStructure.Pull = GPIO_NOPULL;

  //HAL_GPIO_Init(GPIOA, &GPIO_InitStructure);
  //HAL_GPIO_Init(GPIOB, &GPIO_InitStructure);
  HAL_GPIO_Init(GPIOC, &GPIO_InitStructure);
  HAL_GPIO_Init(GPIOH, &GPIO_InitStructure);

  /* Disable GPIOs clock */
  //__HAL_RCC_GPIOA_CLK_DISABLE();
  //__HAL_RCC_GPIOB_CLK_DISABLE();
  __HAL_RCC_GPIOC_CLK_DISABLE();
  __HAL_RCC_GPIOH_CLK_DISABLE();
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
