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

#include "RF24.h"
#include "RF24_mesh.h"
#include "aht20.h"
#include "bmp180.h"
#include "util.h"

#include "mem.h"
#include "eventlog.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
//#define NO_SLEEP

#define SENSOR_FLAG_AHT20_OK            0x01
#define SENSOR_FLAG_BMP180_OK           0x02
#define SENSOR_FLAG_MEM_FULL            0x20
#define SENSOR_FLAG_MEM_OK              0x40
#define SENSOR_FLAG_CONNECTED           0x80

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

I2C_HandleTypeDef hi2c1;

UART_HandleTypeDef hlpuart1;

RTC_HandleTypeDef hrtc;

SPI_HandleTypeDef hspi1;

/* USER CODE BEGIN PV */
uint8_t wait = 0;

typedef struct {
  uint32_t vcc;
  uint16_t period;
  uint8_t connected;
  uint8_t flags;
} sensor_state_t;

sensor_state_t sensor_state;

typedef struct {
    uint32_t timestamp;         //
    uint16_t vcc;
    uint8_t flags;              //flags
    int8_t temp;                //aht20 temp
    uint8_t humidity;           //aht20 hum
    int8_t temp2;               //bmp180 temp
    uint16_t pressure;          //bmp180 pressure
    uint16_t light;             //raw adc from photoresistor
    uint16_t crc16;             //entry checksum
} sensor_data_t;

sensor_data_t s_data;

uint8_t type;
uint8_t cmd[16];
uint32_t tmp32;
uint32_t i;

 //uint32_t const * eeprom_data = (uint32_t const *)0x08080000;
//uint8_t const * const Addr = (uint8_t const *)0x8080000;
/*
typedef struct {
    uint32_t timestamp;         //
    uint16_t vcc;
    uint8_t dummy;              //flags
    int8_t temp;                //aht20 temp
    uint8_t humidity;           //aht20 hum
    int8_t temp2;               //bmp180 temp
    uint16_t pressure;          //bmp180 pressure
    uint16_t light;             //raw adc from photoresistor
    uint16_t crc16;             //entry checksum
} eeprom_entry_t ;

typedef struct {
eeprom_entry_t entries[16];
//uint32_t crc32;
} eeprom_page_t;

eeprom_page_t cache;
uint16_t eeprom_page_ptr;
uint16_t entry_ptr;

eeprom_entry_t temp_entry;
*/

 //W25QXX_HandleTypeDef w25_handle;
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
//uint8_t board_GetNextFreeEntry(void);
//uint8_t board_WriteEntry(eeprom_entry_t *entry);
//uint32_t board_ConvertToUnixTime(RTC_DateTypeDef *psDate, RTC_TimeTypeDef *psTime);

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

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

  sensor_state.vcc = 0;
  sensor_state.period = 15;
  sensor_state.connected = 0;
  sensor_state.flags = 0;

  mesh_Init();

  LL_GPIO_SetOutputPin(GPIOA, GPIO_PIN_11);
  if(eventlog_init( sizeof(sensor_data_t) ) == LOG_OK)
  {
      sensor_state.flags |= SENSOR_FLAG_MEM_OK;
  }

  // нужна задержка, чтобы можно было прошивку записать/ прочитать
  // в режимах сна SWIO отключается
  // в RTC_init ставится будильник с прерыванием на 5 секунд
  LL_GPIO_SetOutputPin(GPIOA, GPIO_PIN_11);
//  wait = 1;
//  while(wait);

  HAL_Delay(1000);
  HAL_Delay(1000);
  HAL_Delay(1000);
  HAL_Delay(1000);
  HAL_Delay(1000);

  LL_GPIO_ResetOutputPin(GPIOA, GPIO_PIN_11);

  // хрен знает поччему, но после повторной инициализции передача идет нормально
  mesh_Init();

  mesh_AddressRequest();
  HAL_Delay(200);
  sensor_state.connected = mesh_Lookup();

  // инициализация датчиков
  if(aht20_Init(&hi2c1) == AHT20_OK)
      sensor_state.flags |= SENSOR_FLAG_AHT20_OK;

  if(bmp180_Init(&hi2c1))
      sensor_state.flags |= SENSOR_FLAG_BMP180_OK;

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

	  // вот тут просыпается микросхема памяти, если спит
	  mem_init();

	  mem_ioctl(MEM_IOCTL_RELEASE, 0);
#endif
	  //собираем всю информацию
	  s_data.vcc = board_GetVcc();
	  s_data.light = board_GetLight();

	  // надо добавиьт внутренний датчик стм в случае отказа оставльных

	  // опрос датчиков
	  if(aht20_Measure() == AHT20_OK)
	  {
	      s_data.temp = (int8_t )(aht20_GetTemp()/10);
	      s_data.humidity = (uint8_t )(aht20_GetHumidity()/10);
	  }
	  else
	  {
	      s_data.temp = -127;
	      s_data.humidity = 255;
	      sensor_state.flags &= ~SENSOR_FLAG_AHT20_OK;
	  }

	  s_data.temp2 = bmp180_GetTemp();
	  s_data.pressure = (uint16_t )(bmp180_GetPressure()/10);

	  s_data.flags = sensor_state.flags;

	  // запуск будильника на следующее пробуждение
	  HAL_RTC_DeactivateAlarm(&hrtc, RTC_ALARM_A);

	  //нужен имеено такй порядок чтения
	  HAL_RTC_GetTime(&hrtc, &sTime, RTC_FORMAT_BIN);
	  HAL_RTC_GetDate(&hrtc, &sDate, RTC_FORMAT_BIN);

	  s_data.timestamp = board_ConvertToUnixTime(&sDate, &sTime);

	  // если не подключены к сети, то пробуем подключиться
	  if(!sensor_state.connected)
	  {
	      sensor_state.connected = mesh_AddressRequest();
	  }

	  // пробуем отправить сообщение
	  if(sensor_state.connected)
	  {
	      sensor_state.connected = mesh_Write('T', (uint8_t *)&s_data, sizeof(sensor_data_t));

	      // ждем чего-нибудь от базовой станции

	      // комнады сброса памяти
	      // запрос данных из памяти
	      // запрос версии по
	      // просто сброс

	      // запрос состояния и флагов устройства
	      // во флеш еще можно писать когда связь отвалилась и появилась

	      if(mesh_Read(&type, cmd, 8))
	      {
	          switch(type)
	          {
                case 'd':
                    HAL_RTC_GetTime(&hrtc, &sTime, RTC_FORMAT_BIN);
                    HAL_RTC_GetDate(&hrtc, &sDate, RTC_FORMAT_BIN);

                    HAL_RTC_SetTime(&hrtc, &sTime, RTC_FORMAT_BIN);
                    sDate.Month = cmd[2]; //month
                    sDate.Date = cmd[1]; //date;
                    sDate.Year = cmd[3];
                    HAL_RTC_SetDate(&hrtc, &sDate, RTC_FORMAT_BIN);
                    break;

                case 't':
                    HAL_RTC_GetTime(&hrtc, &sTime, RTC_FORMAT_BIN);
                    HAL_RTC_GetDate(&hrtc, &sDate, RTC_FORMAT_BIN);

                    sTime.Hours = cmd[1];
                    sTime.Minutes = cmd[2];
                    sTime.Seconds = cmd[3];

                    HAL_RTC_SetTime(&hrtc, &sTime, RTC_FORMAT_BIN);
                    HAL_RTC_SetDate(&hrtc, &sDate, RTC_FORMAT_BIN);
                    break;

                case 'P':
                    memcpy(&(sensor_state.period), &cmd[1], 2);
                    break;

                case 'm':
                    tmp32 = eventlog_getFreeMemory();
                    memcpy(cmd, &tmp32, 4);
                    tmp32 = eventlog_getTotalMemory();
                    memcpy(cmd+4, &tmp32, 4);
                    sensor_state.connected = mesh_Write('m', cmd, 8);
                    break;

                case 's':
                    sensor_state.connected = mesh_Write('s', (uint8_t *)&sensor_state, sizeof(sensor_state));
                    break;

                case 'M':
                    //тут мы посылаем целую страницу памяти из 16 записей
                    // в аргументе функцииномер страницы (всего 4 194 304 байта, страниц 16384 страниц)
                    memcpy(&tmp32, cmd + 1, 4);

                    if(tmp32 > 16384)
                        break;

                    for(i = 0; i < 16; i++)
                    {
                        eventlog_read(i*16 + tmp32, (sensor_data_t *)&s_data);

                        mesh_Write('T', (uint8_t *)&s_data, sizeof(sensor_data_t));
                        HAL_Delay(100);
                    }
                    break;

                case 'G':
                    memcpy(&tmp32, cmd + 1, 4);

                    if(tmp32 > 4194304)
                        break;

                    for(i = 0; i < 16; i++)
                    {
                        eventlog_read(i*16 + tmp32, (sensor_data_t *)&s_data);
                        mesh_Write('T', (uint8_t *)&s_data, sizeof(sensor_data_t));
                        HAL_Delay(100);
                    }
                    break;

                case 'f':
                    eventlog_flush();

                    tmp32 = eventlog_getFreeMemory();
                    memcpy(cmd, &tmp32, 4);
                    tmp32 = eventlog_getTotalMemory();
                    memcpy(cmd+4, &tmp32, 4);
                    sensor_state.connected = mesh_Write('m', cmd, 8);
                    break;
	          }
	      }
	  }

	  //пишем все во флешку
	  if(eventlog_write(&s_data) == LOG_NO_MEM)
	  {
	      sensor_state.flags |= SENSOR_FLAG_MEM_FULL;
	  }

	  // ставим время следующего пробуждения
	    sTime.Minutes = sTime.Minutes + (sensor_state.period / 60);
	    if(sTime.Minutes >= 60)
        {
            sTime.Minutes -= 60;
        }

	    sTime.Seconds = sTime.Seconds + (sensor_state.period % 60);
	    if(sTime.Seconds >= 60)
	    {
	        sTime.Seconds -= 60;
	        sTime.Minutes++;
	        sTime.Minutes %= 60;
	    }

        sAlarm.AlarmTime.Hours = 0;
	    sAlarm.AlarmTime.Minutes = sTime.Minutes;
	    sAlarm.AlarmTime.Seconds = sTime.Seconds;

        sAlarm.AlarmTime.SubSeconds = 0;
        sAlarm.AlarmTime.DayLightSaving = RTC_DAYLIGHTSAVING_NONE;
        sAlarm.AlarmTime.StoreOperation = RTC_STOREOPERATION_RESET;
        sAlarm.AlarmMask = RTC_ALARMMASK_DATEWEEKDAY | RTC_ALARMMASK_HOURS;// |RTC_ALARMMASK_MINUTES;
        sAlarm.AlarmSubSecondMask = RTC_ALARMSUBSECONDMASK_ALL;
        sAlarm.AlarmDateWeekDaySel = RTC_ALARMDATEWEEKDAYSEL_DATE;
        sAlarm.AlarmDateWeekDay = 1;
        sAlarm.Alarm = RTC_ALARM_A;
        if (HAL_RTC_SetAlarm_IT(&hrtc, &sAlarm, RTC_FORMAT_BIN) != HAL_OK)
        {
          Error_Handler();
        }

#ifdef NO_SLEEP
        NRF_sleep();

		wait = 1;
		while(wait);
#else
		NRF_sleep();

		mem_ioctl(MEM_IOCTL_POWERDOWN, 0);

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
  //LL_RCC_SetLPUARTClockSource(LL_RCC_LPUART1_CLKSOURCE_LSE);
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

  LL_ADC_REG_InitTypeDef ADC_REG_InitStruct = {0};
  LL_ADC_InitTypeDef ADC_InitStruct = {0};

  LL_GPIO_InitTypeDef GPIO_InitStruct = {0};

  /* Peripheral clock enable */
  LL_APB2_GRP1_EnableClock(LL_APB2_GRP1_PERIPH_ADC1);

  LL_IOP_GRP1_EnableClock(LL_IOP_GRP1_PERIPH_GPIOA);
  LL_IOP_GRP1_EnableClock(LL_IOP_GRP1_PERIPH_GPIOB);
  /**ADC GPIO Configuration
  PA0-CK_IN   ------> ADC_IN0
  PB1   ------> ADC_IN9
  */
  GPIO_InitStruct.Pin = LL_GPIO_PIN_0;
  GPIO_InitStruct.Mode = LL_GPIO_MODE_ANALOG;
  GPIO_InitStruct.Pull = LL_GPIO_PULL_NO;
  LL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  GPIO_InitStruct.Pin = LL_GPIO_PIN_1;
  GPIO_InitStruct.Mode = LL_GPIO_MODE_ANALOG;
  GPIO_InitStruct.Pull = LL_GPIO_PULL_NO;
  LL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /* USER CODE BEGIN ADC_Init 1 */

  /* USER CODE END ADC_Init 1 */

  /** Configure Regular Channel
  */
  LL_ADC_REG_SetSequencerChAdd(ADC1, LL_ADC_CHANNEL_0);

  /** Configure Regular Channel
  */
  LL_ADC_REG_SetSequencerChAdd(ADC1, LL_ADC_CHANNEL_9);

  /** Common config
  */
  ADC_REG_InitStruct.TriggerSource = LL_ADC_REG_TRIG_SOFTWARE;
  ADC_REG_InitStruct.SequencerDiscont = LL_ADC_REG_SEQ_DISCONT_DISABLE;
  ADC_REG_InitStruct.ContinuousMode = LL_ADC_REG_CONV_SINGLE;
  ADC_REG_InitStruct.DMATransfer = LL_ADC_REG_DMA_TRANSFER_NONE;
  ADC_REG_InitStruct.Overrun = LL_ADC_REG_OVR_DATA_PRESERVED;
  LL_ADC_REG_Init(ADC1, &ADC_REG_InitStruct);
  LL_ADC_SetSamplingTimeCommonChannels(ADC1, LL_ADC_SAMPLINGTIME_79CYCLES_5);
  LL_ADC_SetOverSamplingScope(ADC1, LL_ADC_OVS_DISABLE);
  LL_ADC_REG_SetSequencerScanDirection(ADC1, LL_ADC_REG_SEQ_SCAN_DIR_FORWARD);
  LL_ADC_SetCommonFrequencyMode(__LL_ADC_COMMON_INSTANCE(ADC1), LL_ADC_CLOCK_FREQ_MODE_LOW);
  LL_ADC_DisableIT_EOC(ADC1);
  LL_ADC_DisableIT_EOS(ADC1);
  ADC_InitStruct.Resolution = LL_ADC_RESOLUTION_12B;
  ADC_InitStruct.DataAlignment = LL_ADC_DATA_ALIGN_RIGHT;
  ADC_InitStruct.LowPowerMode = LL_ADC_LP_MODE_NONE;
  LL_ADC_Init(ADC1, &ADC_InitStruct);
  LL_ADC_SetClock(ADC1, LL_ADC_CLOCK_ASYNC);
  LL_ADC_SetCommonClock(__LL_ADC_COMMON_INSTANCE(ADC1), LL_ADC_CLOCK_ASYNC_DIV8);

  /* Enable ADC internal voltage regulator */
  LL_ADC_EnableInternalRegulator(ADC1);
  /* Delay for ADC internal voltage regulator stabilization. */
  /* Compute number of CPU cycles to wait for, from delay in us. */
  /* Note: Variable divided by 2 to compensate partially */
  /* CPU processing cycles (depends on compilation optimization). */
  /* Note: If system core clock frequency is below 200kHz, wait time */
  /* is only a few CPU processing cycles. */
  uint32_t wait_loop_index;
  wait_loop_index = ((LL_ADC_DELAY_INTERNAL_REGUL_STAB_US * (SystemCoreClock / (100000 * 2))) / 10);
  while(wait_loop_index != 0)
  {
    wait_loop_index--;
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
        sTime.Hours = 9;
        sTime.Minutes = 00;
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

  /** Initialize RTC and set the Time and Date
  */
//  sTime.Hours = 12;
//  sTime.Minutes = 15;
//  sTime.Seconds = 0;
//  sTime.DayLightSaving = RTC_DAYLIGHTSAVING_NONE;
//  sTime.StoreOperation = RTC_STOREOPERATION_RESET;
//  if (HAL_RTC_SetTime(&hrtc, &sTime, RTC_FORMAT_BIN) != HAL_OK)
//  {
//    Error_Handler();
//  }
//  sDate.WeekDay = RTC_WEEKDAY_SUNDAY;
//  sDate.Month = RTC_MONTH_JANUARY;
//  sDate.Date = 12;
//  sDate.Year = 25;
//
//  if (HAL_RTC_SetDate(&hrtc, &sDate, RTC_FORMAT_BIN) != HAL_OK)
//  {
//    Error_Handler();
//  }

  /** Enable the Alarm A
  */
//  sAlarm.AlarmTime.Hours = 12;
//  sAlarm.AlarmTime.Minutes = 15;
//  sAlarm.AlarmTime.Seconds = 5;
//  sAlarm.AlarmTime.SubSeconds = 0;
//  sAlarm.AlarmTime.DayLightSaving = RTC_DAYLIGHTSAVING_NONE;
//  sAlarm.AlarmTime.StoreOperation = RTC_STOREOPERATION_RESET;
//  sAlarm.AlarmMask = RTC_ALARMMASK_DATEWEEKDAY|RTC_ALARMMASK_HOURS
//                              |RTC_ALARMMASK_MINUTES;
//  sAlarm.AlarmSubSecondMask = RTC_ALARMSUBSECONDMASK_ALL;
//  sAlarm.AlarmDateWeekDaySel = RTC_ALARMDATEWEEKDAYSEL_DATE;
//  sAlarm.AlarmDateWeekDay = 1;
//  sAlarm.Alarm = RTC_ALARM_A;
//  if (HAL_RTC_SetAlarm_IT(&hrtc, &sAlarm, RTC_FORMAT_BIN) != HAL_OK)
//  {
//    Error_Handler();
//  }
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
  LL_GPIO_ResetOutputPin(GPIOB, LL_GPIO_PIN_7);

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
//  GPIO_InitStruct.Pin = LL_GPIO_PIN_15;
//  GPIO_InitStruct.Mode = LL_GPIO_MODE_INPUT;
//  GPIO_InitStruct.Pull = LL_GPIO_PULL_NO;
//  LL_GPIO_Init(GPIOA, &GPIO_InitStruct);

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

void ConversionStartPoll_ADC_GrpRegular(void)
{
  uint32_t Timeout = 0; /* Variable used for timeout management */

  if ((LL_ADC_IsEnabled(ADC1) == 1)               &&
      (LL_ADC_IsDisableOngoing(ADC1) == 0)        &&
      (LL_ADC_REG_IsConversionOngoing(ADC1) == 0)   )
  {
    LL_ADC_REG_StartConversion(ADC1);
  }


  Timeout = 200; //ms;

  while (LL_ADC_IsActiveFlag_EOC(ADC1) == 0)
  {
    /* Check Systick counter flag to decrement the time-out value */
    if (LL_SYSTICK_IsActiveCounterFlag())
    {
      if(Timeout-- == 0)
      {
          return;
      }
    }
  }

  LL_ADC_ClearFlag_EOC(ADC1);

}

uint16_t board_GetVcc(void)
{
    uint32_t temp;
    uint8_t i;

    LL_ADC_Enable(ADC1);
    LL_GPIO_SetOutputPin(GPIOA, VCC_EN_Pin);
    HAL_Delay(1);

    LL_ADC_REG_SetSequencerChannels(ADC1, LL_ADC_CHANNEL_0);
    temp = 0;
    for(i = 0 ; i < 8; i++)
    {
        ConversionStartPoll_ADC_GrpRegular();
        temp += LL_ADC_REG_ReadConversionData12(ADC1);
        __NOP();

    }
    temp /= 8;

    temp = temp*1553/1000;
    // r1*(r1+r2) = 0.492
    LL_GPIO_ResetOutputPin(GPIOA, VCC_EN_Pin);

    return (uint16_t )temp;
}

uint16_t board_GetLight(void)
{
    uint32_t temp;

    LL_ADC_Enable(ADC1);

    LL_GPIO_ResetOutputPin(GPIOA, LIGHT_EN_Pin);
    HAL_Delay(1);

    LL_ADC_REG_SetSequencerChannels(ADC1, LL_ADC_CHANNEL_9);

    ConversionStartPoll_ADC_GrpRegular();
    temp = LL_ADC_REG_ReadConversionData12(ADC1);

    LL_GPIO_SetOutputPin(GPIOA, LIGHT_EN_Pin);

    return (uint16_t )temp;
}
//
//uint8_t board_GetNextFreeEntry(void)
//{
//    uint8_t found = 0;
//    uint16_t i;
//    //поиск свободной запсиси
//    for(i = 0; (i < 16384) && (!found); i++)
//    {
//        mem_read_page((uint8_t *)&cache, i, 1);
//
//        for(entry_ptr = 0; entry_ptr < 16; entry_ptr++)
//        {
//            if(cache.entries[entry_ptr].crc16 == 0xFFFF)
//            {
//                eeprom_page_ptr = i;
//                found = 1;
//                break;
//            }
//        }
//    }
//
//    return found;
//}
//
//uint8_t board_WriteEntry(eeprom_entry_t *entry)
//{
//    if(eeprom_page_ptr == 16384)
//        return 0;
//
//    if(entry_ptr == 0)
//    {
//        mem_read_page((uint8_t *)&cache, eeprom_page_ptr, 1);
//    }
//
//    memcpy(&(cache.entries[entry_ptr++]), entry, sizeof(eeprom_entry_t));
//
//    if(entry_ptr == 16)
//    {
//        if(eeprom_page_ptr < 16384)
//        {
//            mem_write_page((uint8_t *)&cache, eeprom_page_ptr, 1);
//            //write
//            eeprom_page_ptr++;
//        }
//        entry_ptr = 0;
//    }
//
//    return 1;
//}
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

  GPIO_InitStructure.Pin = 0xFF00 | GPIO_PIN_7;
  GPIO_InitStructure.Mode = GPIO_MODE_ANALOG;
  GPIO_InitStructure.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStructure);

  /* Disable GPIOs clock */
  //__HAL_RCC_GPIOA_CLK_DISABLE();
  __HAL_RCC_GPIOB_CLK_DISABLE();
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
