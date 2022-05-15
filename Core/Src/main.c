/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2022 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  *
  * Company: Jacalt Laser Corp.
  * Author: J. Xu
  * Date: 20220506
  * Version: V1.2.0
  *
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "cmsis_os.h"
#include "lwip.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <string.h>
#include "api.h"
#include "sysinit.h"
#include "cmd.h"
#include "algorithm.h"
#include "communicate.h"
#include "stepmotor.h"
#include "seamfind.h"
#include "tcp_client.h"
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
RTC_HandleTypeDef hrtc;

TIM_HandleTypeDef htim7;

UART_HandleTypeDef huart8;
UART_HandleTypeDef huart3;
DMA_HandleTypeDef hdma_uart8_rx;
DMA_HandleTypeDef hdma_usart3_rx;

/* Definitions for LwipTask */
osThreadId_t LwipTaskHandle;
const osThreadAttr_t LwipTask_attributes = {
  .name = "LwipTask",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityNormal,
};
/* Definitions for SampleTask */
osThreadId_t SampleTaskHandle;
const osThreadAttr_t SampleTask_attributes = {
  .name = "SampleTask",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityLow,
};
/* Definitions for CmdHandleTask */
osThreadId_t CmdHandleTaskHandle;
const osThreadAttr_t CmdHandleTask_attributes = {
  .name = "CmdHandleTask",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityLow,
};
/* Definitions for ProtectTask */
osThreadId_t ProtectTaskHandle;
const osThreadAttr_t ProtectTask_attributes = {
  .name = "ProtectTask",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityLow,
};
/* Definitions for MotorControlTask */
osThreadId_t MotorControlTaskHandle;
const osThreadAttr_t MotorControlTask_attributes = {
  .name = "MotorControlTask",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityLow,
};
/* Definitions for IPCUartTask */
osThreadId_t IPCUartTaskHandle;
const osThreadAttr_t IPCUartTask_attributes = {
  .name = "IPCUartTask",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityLow,
};
/* Definitions for CCDUartTask */
osThreadId_t CCDUartTaskHandle;
const osThreadAttr_t CCDUartTask_attributes = {
  .name = "CCDUartTask",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityLow,
};
/* Definitions for IPCUartQueue */
osMessageQueueId_t IPCUartQueueHandle;
const osMessageQueueAttr_t IPCUartQueue_attributes = {
  .name = "IPCUartQueue"
};
/* Definitions for MotorControlQueue */
osMessageQueueId_t MotorControlQueueHandle;
const osMessageQueueAttr_t MotorControlQueue_attributes = {
  .name = "MotorControlQueue"
};
/* Definitions for CCDUartQueue */
osMessageQueueId_t CCDUartQueueHandle;
const osMessageQueueAttr_t CCDUartQueue_attributes = {
  .name = "CCDUartQueue"
};
/* Definitions for CmdhandleQueue */
osMessageQueueId_t CmdhandleQueueHandle;
const osMessageQueueAttr_t CmdhandleQueue_attributes = {
  .name = "CmdhandleQueue"
};
/* Definitions for LaserEthernetQueue */
osMessageQueueId_t LaserEthernetQueueHandle;
const osMessageQueueAttr_t LaserEthernetQueue_attributes = {
  .name = "LaserEthernetQueue"
};
/* Definitions for TimeCountTimer */
osTimerId_t TimeCountTimerHandle;
const osTimerAttr_t TimeCountTimer_attributes = {
  .name = "TimeCountTimer"
};
/* Definitions for CheckCCDTimer */
osTimerId_t CheckCCDTimerHandle;
const osTimerAttr_t CheckCCDTimer_attributes = {
  .name = "CheckCCDTimer"
};
/* Definitions for IPCUartMutex */
osMutexId_t IPCUartMutexHandle;
const osMutexAttr_t IPCUartMutex_attributes = {
  .name = "IPCUartMutex"
};
/* Definitions for StepMotorXMutex */
osMutexId_t StepMotorXMutexHandle;
const osMutexAttr_t StepMotorXMutex_attributes = {
  .name = "StepMotorXMutex"
};
/* Definitions for StepMotorYMutex */
osMutexId_t StepMotorYMutexHandle;
const osMutexAttr_t StepMotorYMutex_attributes = {
  .name = "StepMotorYMutex"
};
/* Definitions for M24512Mutex */
osMutexId_t M24512MutexHandle;
const osMutexAttr_t M24512Mutex_attributes = {
  .name = "M24512Mutex"
};
/* Definitions for CCDUartMutex */
osMutexId_t CCDUartMutexHandle;
const osMutexAttr_t CCDUartMutex_attributes = {
  .name = "CCDUartMutex"
};
/* Definitions for LaserAlarmBinarySem */
osSemaphoreId_t LaserAlarmBinarySemHandle;
const osSemaphoreAttr_t LaserAlarmBinarySem_attributes = {
  .name = "LaserAlarmBinarySem"
};
/* Definitions for MotorAlarm1BinarySem */
osSemaphoreId_t MotorAlarm1BinarySemHandle;
const osSemaphoreAttr_t MotorAlarm1BinarySem_attributes = {
  .name = "MotorAlarm1BinarySem"
};
/* Definitions for MotorAlarm2BinarySem */
osSemaphoreId_t MotorAlarm2BinarySemHandle;
const osSemaphoreAttr_t MotorAlarm2BinarySem_attributes = {
  .name = "MotorAlarm2BinarySem"
};
/* USER CODE BEGIN PV */
RTC_TimeTypeDef TimeNow;
RTC_DateTypeDef DateNow;

float shtc3_temp;
float shtc3_hum;

uint8_t CCD_rx_buffer[8000];
uint8_t IPC_rx_buffer[DMA_RECV_LEN] = {0};
uint32_t current_welding_length = 0;
uint8_t warning[WARNING_BYTE] = {0};
uint32_t total_welding_length;

uint8_t x_degree;
uint8_t y_degree;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_RTC_Init(void);
static void MX_DMA_Init(void);
static void MX_UART8_Init(void);
static void MX_TIM7_Init(void);
static void MX_USART3_UART_Init(void);
void StartLwipTask(void *argument);
void StartSampleTask(void *argument);
void StartCmdHandleTask(void *argument);
void StartProtectTask(void *argument);
void StartMotorContorlTask(void *argument);
void StartIPCUartTask(void *argument);
void StartCCDUartTask(void *argument);
void TimeCountCallback(void *argument);
void CheckCCDCallback(void *argument);

static void MX_NVIC_Init(void);
/* USER CODE BEGIN PFP */

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
  MX_DMA_Init();
  MX_UART8_Init();
  MX_TIM7_Init();
  MX_USART3_UART_Init();

  /* Initialize interrupts */
  MX_NVIC_Init();
  /* USER CODE BEGIN 2 */

  //-- The hardware initialized successful.
  HAL_UART_Transmit(&huart8, (uint8_t*)"Hello World\r\n", sizeof("Hello World\r\n")-1, 50);
  software_init();    // system initialize
  get_eeprom_data();  // read EEPROM
  check_machine();    // self checking of board and the machines

  /* USER CODE END 2 */

  /* Init scheduler */
  osKernelInitialize();
  /* Create the mutex(es) */
  /* creation of IPCUartMutex */
  IPCUartMutexHandle = osMutexNew(&IPCUartMutex_attributes);

  /* creation of StepMotorXMutex */
  StepMotorXMutexHandle = osMutexNew(&StepMotorXMutex_attributes);

  /* creation of StepMotorYMutex */
  StepMotorYMutexHandle = osMutexNew(&StepMotorYMutex_attributes);

  /* creation of M24512Mutex */
  M24512MutexHandle = osMutexNew(&M24512Mutex_attributes);

  /* creation of CCDUartMutex */
  CCDUartMutexHandle = osMutexNew(&CCDUartMutex_attributes);

  /* USER CODE BEGIN RTOS_MUTEX */
  /* add mutexes, ... */
  /* USER CODE END RTOS_MUTEX */

  /* Create the semaphores(s) */
  /* creation of LaserAlarmBinarySem */
  LaserAlarmBinarySemHandle = osSemaphoreNew(1, 1, &LaserAlarmBinarySem_attributes);

  /* creation of MotorAlarm1BinarySem */
  MotorAlarm1BinarySemHandle = osSemaphoreNew(1, 1, &MotorAlarm1BinarySem_attributes);

  /* creation of MotorAlarm2BinarySem */
  MotorAlarm2BinarySemHandle = osSemaphoreNew(1, 1, &MotorAlarm2BinarySem_attributes);

  /* USER CODE BEGIN RTOS_SEMAPHORES */
  /* add semaphores, ... */
  /* USER CODE END RTOS_SEMAPHORES */

  /* Create the timer(s) */
  /* creation of TimeCountTimer */
  TimeCountTimerHandle = osTimerNew(TimeCountCallback, osTimerPeriodic, NULL, &TimeCountTimer_attributes);

  /* creation of CheckCCDTimer */
  CheckCCDTimerHandle = osTimerNew(CheckCCDCallback, osTimerPeriodic, NULL, &CheckCCDTimer_attributes);

  /* USER CODE BEGIN RTOS_TIMERS */
  /* start timers, add new ones, ... */
  osTimerStart(TimeCountTimerHandle, 1000);  // start time count each 1s
  osTimerStart(CheckCCDTimerHandle, 5000);   // read CCD data each 5s

  /* USER CODE END RTOS_TIMERS */

  /* Create the queue(s) */
  /* creation of IPCUartQueue */
  IPCUartQueueHandle = osMessageQueueNew (1, sizeof(uint64_t), &IPCUartQueue_attributes);

  /* creation of MotorControlQueue */
  MotorControlQueueHandle = osMessageQueueNew (20, sizeof(uint64_t), &MotorControlQueue_attributes);

  /* creation of CCDUartQueue */
  CCDUartQueueHandle = osMessageQueueNew (20, sizeof(uint64_t), &CCDUartQueue_attributes);

  /* creation of CmdhandleQueue */
  CmdhandleQueueHandle = osMessageQueueNew (20, sizeof(uint64_t), &CmdhandleQueue_attributes);

  /* creation of LaserEthernetQueue */
  LaserEthernetQueueHandle = osMessageQueueNew (20, sizeof(uint64_t), &LaserEthernetQueue_attributes);

  /* USER CODE BEGIN RTOS_QUEUES */
  /* add queues, ... */
  /* USER CODE END RTOS_QUEUES */

  /* Create the thread(s) */
  /* creation of LwipTask */
  LwipTaskHandle = osThreadNew(StartLwipTask, NULL, &LwipTask_attributes);

  /* creation of SampleTask */
  SampleTaskHandle = osThreadNew(StartSampleTask, NULL, &SampleTask_attributes);

  /* creation of CmdHandleTask */
  CmdHandleTaskHandle = osThreadNew(StartCmdHandleTask, NULL, &CmdHandleTask_attributes);

  /* creation of ProtectTask */
  ProtectTaskHandle = osThreadNew(StartProtectTask, NULL, &ProtectTask_attributes);

  /* creation of MotorControlTask */
  MotorControlTaskHandle = osThreadNew(StartMotorContorlTask, NULL, &MotorControlTask_attributes);

  /* creation of IPCUartTask */
  IPCUartTaskHandle = osThreadNew(StartIPCUartTask, NULL, &IPCUartTask_attributes);

  /* creation of CCDUartTask */
  CCDUartTaskHandle = osThreadNew(StartCCDUartTask, NULL, &CCDUartTask_attributes);

  /* USER CODE BEGIN RTOS_THREADS */
  /* add threads, ... */
  /* USER CODE END RTOS_THREADS */

  /* USER CODE BEGIN RTOS_EVENTS */
  /* add events, ... */
  /* USER CODE END RTOS_EVENTS */

  /* Start scheduler */
  osKernelStart();

  /* We should never get here as control is now taken by the scheduler */
  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
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

  /** Configure the main internal regulator output voltage
  */
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);
  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE|RCC_OSCILLATORTYPE_LSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.LSEState = RCC_LSE_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 20;
  RCC_OscInitStruct.PLL.PLLN = 360;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 4;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }
  /** Activate the Over-Drive mode
  */
  if (HAL_PWREx_EnableOverDrive() != HAL_OK)
  {
    Error_Handler();
  }
  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV4;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_5) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief NVIC Configuration.
  * @retval None
  */
static void MX_NVIC_Init(void)
{
  /* USART3_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(USART3_IRQn, 5, 0);
  HAL_NVIC_EnableIRQ(USART3_IRQn);
  /* EXTI15_10_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(EXTI15_10_IRQn, 5, 0);
  HAL_NVIC_EnableIRQ(EXTI15_10_IRQn);
  /* TIM7_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(TIM7_IRQn, 5, 0);
  HAL_NVIC_EnableIRQ(TIM7_IRQn);
  /* ETH_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(ETH_IRQn, 5, 0);
  HAL_NVIC_EnableIRQ(ETH_IRQn);
  /* UART8_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(UART8_IRQn, 5, 0);
  HAL_NVIC_EnableIRQ(UART8_IRQn);
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

  /* USER CODE BEGIN RTC_Init 1 */

  /* USER CODE END RTC_Init 1 */
  /** Initialize RTC Only
  */
  hrtc.Instance = RTC;
  hrtc.Init.HourFormat = RTC_HOURFORMAT_24;
  hrtc.Init.AsynchPrediv = 127;
  hrtc.Init.SynchPrediv = 255;
  hrtc.Init.OutPut = RTC_OUTPUT_DISABLE;
  hrtc.Init.OutPutPolarity = RTC_OUTPUT_POLARITY_HIGH;
  hrtc.Init.OutPutType = RTC_OUTPUT_TYPE_OPENDRAIN;
  if (HAL_RTC_Init(&hrtc) != HAL_OK)
  {
    Error_Handler();
  }

  /* USER CODE BEGIN Check_RTC_BKUP */

  /* USER CODE END Check_RTC_BKUP */

  /** Initialize RTC and set the Time and Date
  */
  sTime.Hours = 0;
  sTime.Minutes = 0;
  sTime.Seconds = 0;
  sTime.DayLightSaving = RTC_DAYLIGHTSAVING_NONE;
  sTime.StoreOperation = RTC_STOREOPERATION_RESET;
  if (HAL_RTC_SetTime(&hrtc, &sTime, RTC_FORMAT_BIN) != HAL_OK)
  {
    Error_Handler();
  }
  sDate.WeekDay = RTC_WEEKDAY_MONDAY;
  sDate.Month = RTC_MONTH_JANUARY;
  sDate.Date = 1;
  sDate.Year = 0;

  if (HAL_RTC_SetDate(&hrtc, &sDate, RTC_FORMAT_BIN) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN RTC_Init 2 */

  /* USER CODE END RTC_Init 2 */

}

/**
  * @brief TIM7 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM7_Init(void)
{

  /* USER CODE BEGIN TIM7_Init 0 */

  /* USER CODE END TIM7_Init 0 */

  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM7_Init 1 */

  /* USER CODE END TIM7_Init 1 */
  htim7.Instance = TIM7;
  htim7.Init.Prescaler = 9000;
  htim7.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim7.Init.Period = 10000;
  htim7.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE;
  if (HAL_TIM_Base_Init(&htim7) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim7, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM7_Init 2 */

  HAL_TIM_Base_Start_IT(&INIT_TIM);  // start the timer

  /* USER CODE END TIM7_Init 2 */

}

/**
  * @brief UART8 Initialization Function
  * @param None
  * @retval None
  */
static void MX_UART8_Init(void)
{

  /* USER CODE BEGIN UART8_Init 0 */

  /* USER CODE END UART8_Init 0 */

  /* USER CODE BEGIN UART8_Init 1 */

  /* USER CODE END UART8_Init 1 */
  huart8.Instance = UART8;
  huart8.Init.BaudRate = 115200;
  huart8.Init.WordLength = UART_WORDLENGTH_8B;
  huart8.Init.StopBits = UART_STOPBITS_1;
  huart8.Init.Parity = UART_PARITY_NONE;
  huart8.Init.Mode = UART_MODE_TX_RX;
  huart8.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart8.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart8) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN UART8_Init 2 */
  //-- idle interrupt and DMA receive
  __HAL_UART_ENABLE_IT(&IPC_UART, UART_IT_IDLE);
  HAL_UART_Receive_DMA(&IPC_UART, IPC_rx_buffer, sizeof(IPC_rx_buffer));

  /* USER CODE END UART8_Init 2 */

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
  if (HAL_UART_Init(&huart3) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART3_Init 2 */

  __HAL_UART_ENABLE_IT(&CCD_UART, UART_IT_IDLE);
  HAL_UART_Receive_DMA(&CCD_UART, CCD_rx_buffer, sizeof(CCD_rx_buffer));

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
  /* DMA1_Stream1_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Stream1_IRQn, 5, 0);
  HAL_NVIC_EnableIRQ(DMA1_Stream1_IRQn);
  /* DMA1_Stream6_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Stream6_IRQn, 5, 0);
  HAL_NVIC_EnableIRQ(DMA1_Stream6_IRQn);

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOE_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(WELD_START_GPIO_Port, WELD_START_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(MOTOR_PU1_GPIO_Port, MOTOR_PU1_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, MOTOR_PU2_Pin|EEPROM_WC_Pin|EEPROM_SCL_Pin|EEPROM_SDA_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOE, SHTC3_SDA_Pin|SHTC3_SCL_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOD, MOTOR_EN2_Pin|MOTOR_DR2_Pin|MOTOR_EN1_Pin|MOTOR_DR1_Pin
                          |LASER_INTERLOCK_Pin|LINE_LASER_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(ETH_NRST_GPIO_Port, ETH_NRST_Pin, GPIO_PIN_SET);

  /*Configure GPIO pin : WELD_START_Pin */
  GPIO_InitStruct.Pin = WELD_START_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(WELD_START_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : MOTOR_PU1_Pin */
  GPIO_InitStruct.Pin = MOTOR_PU1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(MOTOR_PU1_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : MOTOR_PU2_Pin ETH_NRST_Pin EEPROM_WC_Pin EEPROM_SCL_Pin
                           EEPROM_SDA_Pin */
  GPIO_InitStruct.Pin = MOTOR_PU2_Pin|ETH_NRST_Pin|EEPROM_WC_Pin|EEPROM_SCL_Pin
                          |EEPROM_SDA_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pins : SHTC3_SDA_Pin SHTC3_SCL_Pin */
  GPIO_InitStruct.Pin = SHTC3_SDA_Pin|SHTC3_SCL_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOE, &GPIO_InitStruct);

  /*Configure GPIO pins : LIMIT_SW1_Pin LIMIT_SW2_Pin */
  GPIO_InitStruct.Pin = LIMIT_SW1_Pin|LIMIT_SW2_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOE, &GPIO_InitStruct);

  /*Configure GPIO pins : MOTOR_EN2_Pin MOTOR_DR2_Pin MOTOR_EN1_Pin MOTOR_DR1_Pin
                           LASER_INTERLOCK_Pin LINE_LASER_Pin */
  GPIO_InitStruct.Pin = MOTOR_EN2_Pin|MOTOR_DR2_Pin|MOTOR_EN1_Pin|MOTOR_DR1_Pin
                          |LASER_INTERLOCK_Pin|LINE_LASER_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);

  /*Configure GPIO pin : LASER_ERROR_Pin */
  GPIO_InitStruct.Pin = LASER_ERROR_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(LASER_ERROR_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : MOTOR_ALM1_Pin MOTOR_ALM2_Pin */
  GPIO_InitStruct.Pin = MOTOR_ALM1_Pin|MOTOR_ALM2_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);

}

/* USER CODE BEGIN 4 */
/******************************************************************************
 * User Task functions
 ******************************************************************************/
/* USER CODE END 4 */

/* USER CODE BEGIN Header_StartLwipTask */
/**
  * @brief  Function implementing the LwipTask thread.
  * @param  argument: Not used
  * @retval None
  *
  * Function: initialize LwIP protocol and connected to the laser
  *           IP address of board is set in MX_LWIP_INIT()
  */
/* USER CODE END Header_StartLwipTask */
void StartLwipTask(void *argument)
{
  /* init code for LWIP */
  MX_LWIP_Init();
  /* USER CODE BEGIN 5 */
  uint8_t msg_prio = 0;
  uint8_t cmd_len;
  uint8_t recv_data[DMA_RECV_LEN] = {0};
  struct netconn *laser_conn;

  laser_conn = laser_tcp_init();

  /* Infinite loop */
  for(;;)
  {
	  if(osOK == osMessageQueueGet((osMessageQueueId_t)LaserEthernetQueueHandle,
				 				    (void *)recv_data,
								    (uint8_t *)&msg_prio,
								    (uint32_t)osWaitForever))
	  {
		  cmd_len = recv_data[2];
		  laser_tcp_comm(recv_data, cmd_len, laser_conn);
	  }

	  osDelay(100);
  }
  /* USER CODE END 5 */
}

/* USER CODE BEGIN Header_StartSampleTask */
/**
* @brief Function implementing the SampleTask thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_StartSampleTask */
void StartSampleTask(void *argument)
{
  /* USER CODE BEGIN StartSampleTask */
  /* Infinite loop */
  for(;;)
  {
    osDelay(1);
  }
  /* USER CODE END StartSampleTask */
}

/* USER CODE BEGIN Header_StartCmdHandleTask */
/**
* @brief Function implementing the CmdHandleTask thread.
* @param argument: Not used
* @retval None
*
* Function: call command parsing program
*/
/* USER CODE END Header_StartCmdHandleTask */
void StartCmdHandleTask(void *argument)
{
  /* USER CODE BEGIN StartCmdHandleTask */
	uint8_t msg_prio = 0;
	uint8_t recv_data[DMA_RECV_LEN] = {0};
  /* Infinite loop */
  for (;;)
  {
	  if(osOK == osMessageQueueGet((osMessageQueueId_t)CmdhandleQueueHandle,
				 				    (void *)recv_data,
								    (uint8_t *)&msg_prio,
								    (uint32_t)osWaitForever))
	  {
		  Cmd_Handle(recv_data, recv_data[2]+3);
	  }
    osDelay(1);
  }
  /* USER CODE END StartCmdHandleTask */
}

/* USER CODE BEGIN Header_StartProtectTask */
/**
* @brief Function implementing the ProtectTask thread.
* @param argument: Not used
* @retval None
*
* Function: set warning bits of the stepmotors when they in alarm status.
*           When the master asks the warning information.
* Warning information: 2x8 byte W1W2, where W2 is not used.
*                      X motor error: B0000 0001
*                      Y motor error: B0000 0010
*/
/* USER CODE END Header_StartProtectTask */
void StartProtectTask(void *argument)
{
  /* USER CODE BEGIN StartProtectTask */
  /* Infinite loop */
	 for (;;)
	  {
		 //-- X stepmoter
		 if (HAL_GPIO_ReadPin(MOTOR_ALM1_GPIO_Port, MOTOR_ALM1_Pin) == GPIO_PIN_SET)
			  warning[0] |= STEPMOTOR_X_WARNING_BIT;
		 else
			  warning[0] &= ~STEPMOTOR_X_WARNING_BIT;

		 //-- Y stepmotor
		  if (HAL_GPIO_ReadPin(MOTOR_ALM2_GPIO_Port, MOTOR_ALM2_Pin) == GPIO_PIN_SET)
			  warning[0] |= STEPMOTOR_Y_WARNING_BIT;
		  else
			  warning[0] &= ~STEPMOTOR_Y_WARNING_BIT;

	      osDelay(100);
	  }
  /* USER CODE END StartProtectTask */
}

/* USER CODE BEGIN Header_StartMotorContorlTask */
/**
* @brief Function implementing the MotorControlTask thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_StartMotorContorlTask */
void StartMotorContorlTask(void *argument)
{
  /* USER CODE BEGIN StartMotorContorlTask */
	typedef_stepmotor_para stepmotor_para = {0};
	uint8_t msg_prio = 0;

  /* Infinite loop */
	  for (;;)
	  {
		  if (osOK == osMessageQueueGet((osMessageQueueId_t)MotorControlQueueHandle,
									    (void *)&stepmotor_para,
									    (uint8_t *)&msg_prio,
									    (uint32_t)osWaitForever))
		  {
			  switch (stepmotor_para.stepmotor)
			  {
			  case STEPMOTOR_X:
				  // TODO check warning bits of stepmotor
				  stepmotor_x_run(stepmotor_para.direction, stepmotor_para.degree);
				  break;

			  case STEPMOTOR_Y:
				  // TODO check warning bits of stepmotor
				  stepmotor_y_run(stepmotor_para.direction, stepmotor_para.degree);
				  break;

			  default:
				  break;
			  }
		  }
		  osDelay(1);
	  }
  /* USER CODE END StartMotorContorlTask */
}

/* USER CODE BEGIN Header_StartIPCUartTask */
/**
* @brief Function implementing the IPCUartTask thread.
* @param argument: Not used
* @retval None
*
* Function: sort the command from the master.
*
*/
/* USER CODE END Header_StartIPCUartTask */
void StartIPCUartTask(void *argument)
{
  /* USER CODE BEGIN StartIPCUartTask */
	uint8_t msg_prio;
	uint8_t recv_data[DMA_RECV_LEN] = {0};
	uint8_t cmd_len;  // command length
	osStatus_t status;

  /* Infinite loop */
  for (;;)
  {
	  status = osMessageQueueGet((osMessageQueueId_t)IPCUartQueueHandle,
			  	  	  	  		 (void *)recv_data,
								 (uint8_t *)&msg_prio,
								 (uint32_t)osWaitForever);
	  if (status == osOK)
	  {
		  if (recv_data[0] == LASER_COMMEND_HEAD1 &&
			  recv_data[1] == LASER_COMMEND_HEAD2)  // of the laser commands
		  {
			  osMessageQueuePut((osMessageQueueId_t)LaserEthernetQueueHandle,
			  					(const void *)recv_data,
			  					(uint8_t)msg_prio,
			  					(uint32_t)osWaitForever);
		  }
		  else if (recv_data[0] == BOARD_COMMEND_HEAD1 &&
				   recv_data[1] == BOARD_COMMEND_HEAD2)  // of the board commands
		  {
			  cmd_len = recv_data[2];
			  if (recv_data[3] == BOARD_ADDRESS &&
				  get_add(recv_data, cmd_len+2) == recv_data[cmd_len+2])
			  {
				  osMessageQueuePut((osMessageQueueId_t)CmdhandleQueueHandle,
				  					(const void *)recv_data,
				  					(uint8_t)msg_prio,
				  					(uint32_t)osWaitForever);
			  }
		  }
	  }
    osDelay(1);
  }
  /* USER CODE END StartIPCUartTask */
}

/* USER CODE BEGIN Header_StartCCDUartTask */
/**
* @brief Function implementing the CCDUartTask thread.
* @param argument: Not used
* @retval None
*
* Function: check the data header from CCD,
*           and call seam_find subprogram
*/
/* USER CODE END Header_StartCCDUartTask */
void StartCCDUartTask(void *argument)
{
  /* USER CODE BEGIN StartCCDUartTask */
	uint8_t msg_prio;
	uint8_t recv_data[DMA_RECV_LEN];
	uint16_t recv_len;
	osStatus_t status;
	uint16_t i;
	uint16_t slot_location[3648]; // data of slot to be welding

  /* Infinite loop */
	for (;;)
	{
		status = osMessageQueueGet((osMessageQueueId_t)CCDUartQueueHandle,
	  	  	  	 	               (void *)recv_data,
				                   (uint8_t *)&msg_prio,
				                   (uint32_t)osWaitForever);
		if (status == osOK)
		{
		  if (recv_data[0]==0x3c && recv_data[1]==0xc3 &&
			  recv_data[2]==0x33 && recv_data[3]==0xcc)  // CCD header check
			{
				recv_len = osMessageQueueGetMsgSize(CCDUartQueueHandle);

				//-- pass through to the master computer
				IPC_UART_SendData(recv_data, recv_len);

				//-- collect the data of slot.
				// each pixel contains 2 bytes, in which it used the lower 4 bits
				//   in the first byte and all 8 bits in the second byte.
				for (i=4; i<recv_len-2; i+=2)
				{
					slot_location[i/2-2] = (recv_data[i]<<4) | recv_data[i+1];
				}
				//-- find the slot by the Gaussian fitting
				find_seam(slot_location);
			}
		}

	  osDelay(100);
	}
  /* USER CODE END StartCCDUartTask */
}

/* TimeCountCallback function */
void TimeCountCallback(void *argument)
{
  /* USER CODE BEGIN TimeCountCallback */
	//-- get current time and data from system clock.
	HAL_RTC_GetTime(&hrtc, &TimeNow, RTC_FORMAT_BIN);
	HAL_RTC_GetDate(&hrtc, &DateNow, RTC_FORMAT_BIN);

  /* USER CODE END TimeCountCallback */
}

/* CheckCCDCallback function */
void CheckCCDCallback(void *argument)
{
  /* USER CODE BEGIN CheckCCDCallback */
  //-- read data from CCD, the command to ask CCD data is
  //   @c0080#@ in ASCII code, defined by the TCD 1304CCD driver.

  uint8_t check_ccd_cmd[8];

  check_ccd_cmd[0] = 0x40;
  check_ccd_cmd[1] = 0x63;
  check_ccd_cmd[2] = 0x30;
  check_ccd_cmd[3] = 0x30;
  check_ccd_cmd[4] = 0x38;
  check_ccd_cmd[5] = 0x23;
  check_ccd_cmd[6] = 0x30;
  check_ccd_cmd[7] = 0x40;

  CCD_UART_SendData(check_ccd_cmd, sizeof(check_ccd_cmd));
  /* USER CODE END CheckCCDCallback */
}

/**
  * @brief  Period elapsed callback in non blocking mode
  * @note   This function is called  when TIM6 interrupt took place, inside
  * HAL_TIM_IRQHandler(). It makes a direct call to HAL_IncTick() to increment
  * a global variable "uwTick" used as application time base.
  * @param  htim : TIM handle
  * @retval None
  */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
  /* USER CODE BEGIN Callback 0 */

  /* USER CODE END Callback 0 */
  if (htim->Instance == TIM6) {
    HAL_IncTick();
  }
  /* USER CODE BEGIN Callback 1 */
  else if (htim->Instance == TIM7)
  {
    // TODO LED lights periodically
  }
  /* USER CODE END Callback 1 */
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

