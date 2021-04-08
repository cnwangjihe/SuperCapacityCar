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
// typedef struct {
// 	uint8_t *data;
//   size_t len;
//   uint8_t id;
// }ACKRequest;
#define UART_BUF_SIZE 0x200
typedef struct {
	uint8_t *data;
  size_t len;
  uint8_t id;
}UDPRequest;

typedef struct {
  uint8_t s[UART_BUF_SIZE];
  size_t len;
}UARTPackage;


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
#define LED0_Pin GPIO_PIN_9
#define LED0_GPIO_Port GPIOF
#define LED1_Pin GPIO_PIN_10
#define LED1_GPIO_Port GPIOF
#define OLED_CS_Pin GPIO_PIN_0
#define OLED_CS_GPIO_Port GPIOC
#define ChargerCtrl_Pin GPIO_PIN_3
#define ChargerCtrl_GPIO_Port GPIOC
#define OLED_DC_Pin GPIO_PIN_15
#define OLED_DC_GPIO_Port GPIOD
#define DP_CLK_Pin GPIO_PIN_11
#define DP_CLK_GPIO_Port GPIOC
#define DP_DIO_Pin GPIO_PIN_12
#define DP_DIO_GPIO_Port GPIOC
#define OLED_Res_Pin GPIO_PIN_4
#define OLED_Res_GPIO_Port GPIOD
/* USER CODE BEGIN Private defines */
#define NETWORK_SEND_OK 1
#define NETWORK_SEND_FAILED 0
#define QOS_ACK 1
#define QOS_NONE 0
#define MAX_CAP_VOL 10
#define MIN_CAP_VOL 2
#define RECIEVE_QUEUE_MAX 0x80
#define UDP_NOT_READY 0
#define UDP_READY 1
#define NETWORK_NOT_READY 0
#define NETWORK_READY 1
#define ESP_NON 0
#define ESP_CWJ 1
#define ESP_GOK 2
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
