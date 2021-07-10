/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2021 STMicroelectronics.
  * All rights reserved.</center></h2>
  *
  * This software component is licensed by ST under Ultimate Liberty license
  * SLA0044, the "License"; You may not use this file except in compliance with
  * the License. You may obtain a copy of the License at:
  *                             www.st.com/SLA0044
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
#include "stm32f4xx_hal.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "retarget.h"
/* USER CODE END Includes */

/* Exported types ------------------------------------------------------------*/
/* USER CODE BEGIN ET */

typedef struct {
  uint16_t data[64];
}AMGData;

typedef struct {
	uint8_t *data;
  size_t len;
  uint8_t id;
}UDPRequest;

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
#define K1_Pin GPIO_PIN_3
#define K1_GPIO_Port GPIOE
#define K0_Pin GPIO_PIN_4
#define K0_GPIO_Port GPIOE
#define ChargerCtrl_Pin GPIO_PIN_8
#define ChargerCtrl_GPIO_Port GPIOF
#define LED0_Pin GPIO_PIN_9
#define LED0_GPIO_Port GPIOF
#define LED1_Pin GPIO_PIN_10
#define LED1_GPIO_Port GPIOF
#define RES_Pin GPIO_PIN_14
#define RES_GPIO_Port GPIOB
#define DC_Pin GPIO_PIN_15
#define DC_GPIO_Port GPIOB
#define CS_Pin GPIO_PIN_8
#define CS_GPIO_Port GPIOD
#define ESP_RST_Pin GPIO_PIN_8
#define ESP_RST_GPIO_Port GPIOA
#define L1_Pin GPIO_PIN_0
#define L1_GPIO_Port GPIOD
#define L3_Pin GPIO_PIN_1
#define L3_GPIO_Port GPIOD
#define L2_Pin GPIO_PIN_2
#define L2_GPIO_Port GPIOD
#define L4_Pin GPIO_PIN_3
#define L4_GPIO_Port GPIOD
#define IN2_Pin GPIO_PIN_6
#define IN2_GPIO_Port GPIOB
#define IN4_Pin GPIO_PIN_7
#define IN4_GPIO_Port GPIOB
#define IN3_Pin GPIO_PIN_8
#define IN3_GPIO_Port GPIOB
#define IN1_Pin GPIO_PIN_9
#define IN1_GPIO_Port GPIOB
/* USER CODE BEGIN Private defines */
#define REP0(X)
#define REP1(X) X
#define REP2(X) REP1(X) X
#define REP3(X) REP2(X) X
#define REP4(X) REP3(X) X
#define REP5(X) REP4(X) X
#define REP6(X) REP5(X) X
#define REP7(X) REP6(X) X
#define REP8(X) REP7(X) X
#define REP9(X) REP8(X) X
#define REP10(X) REP9(X) X

#define REP(HUNDREDS,TENS,ONES,X) \
  REP##HUNDREDS(REP10(REP10(X))) \
  REP##TENS(REP10(X)) \
  REP##ONES(X)

#define MAX(a,b) ((a) > (b) ? (a) : (b))
#define MIN(a,b) ((a) < (b) ? (a) : (b))

#define UART_BUF_SIZE 0x300

#define NETWORK_SEND_OK 1
#define NETWORK_SEND_FAILED 0
#define QOS_ACK 1
#define QOS_NONE 0
#define MAX_CAP_VOL 10
#define MIN_CAP_VOL 2
#define MOTOR_STOPPED 0
#define MOTOR_RUNNING 1
#define MOTOR_STARTED 2
#define RECEIVE_QUEUE_MAX 0x80
#define ITMassert(x,y) if ((x) == 0) {printf("%s",(y));}
#define DEBUG
#ifdef DEBUG
  #define itm_printf printf
#else
  #define itm_printf (void)sizeof
#endif
/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
