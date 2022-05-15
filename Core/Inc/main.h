/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
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

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __MAIN_H
#define __MAIN_H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "stm32f4xx_hal.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "cmsis_os.h"
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
#define WELD_START_Pin GPIO_PIN_0
#define WELD_START_GPIO_Port GPIOC
#define MOTOR_PU1_Pin GPIO_PIN_3
#define MOTOR_PU1_GPIO_Port GPIOA
#define MOTOR_PU2_Pin GPIO_PIN_0
#define MOTOR_PU2_GPIO_Port GPIOB
#define SHTC3_SDA_Pin GPIO_PIN_11
#define SHTC3_SDA_GPIO_Port GPIOE
#define SHTC3_SCL_Pin GPIO_PIN_12
#define SHTC3_SCL_GPIO_Port GPIOE
#define LIMIT_SW1_Pin GPIO_PIN_13
#define LIMIT_SW1_GPIO_Port GPIOE
#define LIMIT_SW2_Pin GPIO_PIN_14
#define LIMIT_SW2_GPIO_Port GPIOE
#define MOTOR_EN2_Pin GPIO_PIN_9
#define MOTOR_EN2_GPIO_Port GPIOD
#define MOTOR_DR2_Pin GPIO_PIN_10
#define MOTOR_DR2_GPIO_Port GPIOD
#define MOTOR_EN1_Pin GPIO_PIN_11
#define MOTOR_EN1_GPIO_Port GPIOD
#define MOTOR_DR1_Pin GPIO_PIN_12
#define MOTOR_DR1_GPIO_Port GPIOD
#define LASER_ERROR_Pin GPIO_PIN_14
#define LASER_ERROR_GPIO_Port GPIOD
#define LASER_ERROR_EXTI_IRQn EXTI15_10_IRQn
#define LASER_INTERLOCK_Pin GPIO_PIN_15
#define LASER_INTERLOCK_GPIO_Port GPIOD
#define CCD_TX_Pin GPIO_PIN_10
#define CCD_TX_GPIO_Port GPIOC
#define CCD_RX_Pin GPIO_PIN_11
#define CCD_RX_GPIO_Port GPIOC
#define MOTOR_ALM1_Pin GPIO_PIN_0
#define MOTOR_ALM1_GPIO_Port GPIOD
#define MOTOR_ALM2_Pin GPIO_PIN_1
#define MOTOR_ALM2_GPIO_Port GPIOD
#define LINE_LASER_Pin GPIO_PIN_7
#define LINE_LASER_GPIO_Port GPIOD
#define ETH_NRST_Pin GPIO_PIN_5
#define ETH_NRST_GPIO_Port GPIOB
#define EEPROM_WC_Pin GPIO_PIN_7
#define EEPROM_WC_GPIO_Port GPIOB
#define EEPROM_SCL_Pin GPIO_PIN_8
#define EEPROM_SCL_GPIO_Port GPIOB
#define EEPROM_SDA_Pin GPIO_PIN_9
#define EEPROM_SDA_GPIO_Port GPIOB
#define IPC_RS232_RX_Pin GPIO_PIN_0
#define IPC_RS232_RX_GPIO_Port GPIOE
#define IPC_RS232_TX_Pin GPIO_PIN_1
#define IPC_RS232_TX_GPIO_Port GPIOE
/* USER CODE BEGIN Private defines */

#define IPC_UART huart8     // master serial port
#define CCD_UART huart3     // CCD serial port
#define INIT_TIM htim7      // initial Timer 7
#define DMA_RECV_LEN 100    // DMA length
#define WARNING_BYTE 2      // bits of warning information

//-- system defined variables for external calling
extern UART_HandleTypeDef CCD_UART;
extern UART_HandleTypeDef IPC_UART;

extern osMutexId_t IPCUartMutexHandle;
extern osMutexId_t CCDUartMutexHandle;
extern osMutexId_t M24512MutexHandle;
extern osMutexId_t StepMotorXMutexHandle;
extern osMutexId_t StepMotorYMutexHandle;

extern osMessageQueueId_t IPCUartQueueHandle;
extern osMessageQueueId_t LaserEthernetQueueHandle;
extern osMessageQueueId_t MotorControlQueueHandle;
extern osMessageQueueId_t CCDUartQueueHandle;

//-- user defined Global variables
extern RTC_TimeTypeDef TimeNow;
extern RTC_DateTypeDef DateNow;
extern TIM_HandleTypeDef htim7;

extern float shtc3_temp;
extern float shtc3_hum;

extern uint8_t CCD_rx_buffer[8000];
extern uint8_t IPC_rx_buffer[DMA_RECV_LEN];
extern uint32_t current_welding_length;
extern uint8_t warning[WARNING_BYTE];

extern uint32_t total_welding_length;
extern uint8_t x_degree;
extern uint8_t y_degree;
/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
