/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    stm32f4xx_it.c
  * @brief   Interrupt Service Routines.
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

/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "stm32f4xx_it.h"
/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "FreeRTOS.h"
#include "task.h"
#include "cmsis_os.h"
#include "stream_buffer.h"
#include "message_buffer.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN TD */

/* USER CODE END TD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define USART_RECEIVE_PENDING 0
#define USART_RECEIVE_RUNNING 1
#define USART_RECEIVE_OVERFLW 2
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
/* USER CODE BEGIN PV */
// extern UARTPackage ReceivedPackage[RECEIVE_QUEUE_MAX];
// extern uint8_t ReceivedPackagePos;
extern MessageBufferHandle_t ReceiveQueueHandle, ESPRetQueueHandle;
uint8_t usart1_status;
size_t BufferLen = 0;
uint8_t Buffer[UART_BUF_SIZE];
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

/* External variables --------------------------------------------------------*/
extern DMA_HandleTypeDef hdma_adc1;
extern DMA_HandleTypeDef hdma_spi1_tx;
extern TIM_HandleTypeDef htim9;
extern UART_HandleTypeDef huart1;
extern TIM_HandleTypeDef htim6;

/* USER CODE BEGIN EV */

/* USER CODE END EV */

/******************************************************************************/
/*           Cortex-M4 Processor Interruption and Exception Handlers          */
/******************************************************************************/
/**
  * @brief This function handles Non maskable interrupt.
  */
void NMI_Handler(void)
{
  /* USER CODE BEGIN NonMaskableInt_IRQn 0 */

  /* USER CODE END NonMaskableInt_IRQn 0 */
  HAL_RCC_NMI_IRQHandler();
  /* USER CODE BEGIN NonMaskableInt_IRQn 1 */
  while (1)
  {
  }
  /* USER CODE END NonMaskableInt_IRQn 1 */
}

/**
  * @brief This function handles Hard fault interrupt.
  */
void HardFault_Handler(void)
{
  /* USER CODE BEGIN HardFault_IRQn 0 */

  /* USER CODE END HardFault_IRQn 0 */
  while (1)
  {
    /* USER CODE BEGIN W1_HardFault_IRQn 0 */
    /* USER CODE END W1_HardFault_IRQn 0 */
  }
}

/**
  * @brief This function handles Memory management fault.
  */
void MemManage_Handler(void)
{
  /* USER CODE BEGIN MemoryManagement_IRQn 0 */

  /* USER CODE END MemoryManagement_IRQn 0 */
  while (1)
  {
    /* USER CODE BEGIN W1_MemoryManagement_IRQn 0 */
    /* USER CODE END W1_MemoryManagement_IRQn 0 */
  }
}

/**
  * @brief This function handles Pre-fetch fault, memory access fault.
  */
void BusFault_Handler(void)
{
  /* USER CODE BEGIN BusFault_IRQn 0 */

  /* USER CODE END BusFault_IRQn 0 */
  while (1)
  {
    /* USER CODE BEGIN W1_BusFault_IRQn 0 */
    /* USER CODE END W1_BusFault_IRQn 0 */
  }
}

/**
  * @brief This function handles Undefined instruction or illegal state.
  */
void UsageFault_Handler(void)
{
  /* USER CODE BEGIN UsageFault_IRQn 0 */

  /* USER CODE END UsageFault_IRQn 0 */
  while (1)
  {
    /* USER CODE BEGIN W1_UsageFault_IRQn 0 */
    /* USER CODE END W1_UsageFault_IRQn 0 */
  }
}

/**
  * @brief This function handles Debug monitor.
  */
void DebugMon_Handler(void)
{
  /* USER CODE BEGIN DebugMonitor_IRQn 0 */

  /* USER CODE END DebugMonitor_IRQn 0 */
  /* USER CODE BEGIN DebugMonitor_IRQn 1 */

  /* USER CODE END DebugMonitor_IRQn 1 */
}

/******************************************************************************/
/* STM32F4xx Peripheral Interrupt Handlers                                    */
/* Add here the Interrupt Handlers for the used peripherals.                  */
/* For the available peripheral interrupt handler names,                      */
/* please refer to the startup file (startup_stm32f4xx.s).                    */
/******************************************************************************/

/**
  * @brief This function handles EXTI line3 interrupt.
  */
void EXTI3_IRQHandler(void)
{
  /* USER CODE BEGIN EXTI3_IRQn 0 */

  /* USER CODE END EXTI3_IRQn 0 */
  HAL_GPIO_EXTI_IRQHandler(GPIO_PIN_3);
  /* USER CODE BEGIN EXTI3_IRQn 1 */

  /* USER CODE END EXTI3_IRQn 1 */
}

/**
  * @brief This function handles TIM1 break interrupt and TIM9 global interrupt.
  */
void TIM1_BRK_TIM9_IRQHandler(void)
{
  /* USER CODE BEGIN TIM1_BRK_TIM9_IRQn 0 */

  /* USER CODE END TIM1_BRK_TIM9_IRQn 0 */
  HAL_TIM_IRQHandler(&htim9);
  /* USER CODE BEGIN TIM1_BRK_TIM9_IRQn 1 */

  /* USER CODE END TIM1_BRK_TIM9_IRQn 1 */
}

/**
  * @brief This function handles USART1 global interrupt.
  */
void USART1_IRQHandler(void)
{
  /* USER CODE BEGIN USART1_IRQn 0 */
  // itm_printf("az");

  if ((__HAL_UART_GET_FLAG(&huart1, UART_FLAG_RXNE) != RESET))
  {
    if (usart1_status == USART_RECEIVE_PENDING)
    {
      usart1_status = USART_RECEIVE_RUNNING;
      __HAL_UART_ENABLE_IT(&huart1, UART_IT_IDLE);
    }
    if (BufferLen == UART_BUF_SIZE)
    {
      usart1_status = USART_RECEIVE_OVERFLW;
      BufferLen = 0; // drop
      if (Buffer[UART_BUF_SIZE-1]=='\r')
        Buffer[BufferLen++] = '\r';
      itm_printf("Shit!Recived line tooooooo looooong\n");
    }
    Buffer[BufferLen++] = (uint8_t)(huart1.Instance->DR & (uint8_t)0x00FF);
    // itm_printf("%02x#",Buffer[BufferLen-1]);
    if (BufferLen == 1 && Buffer[0]=='>')
    {
      xMessageBufferSendFromISR(ESPRetQueueHandle,Buffer,BufferLen,NULL);
      usart1_status = USART_RECEIVE_RUNNING;
      BufferLen = 0;
    }
    else if (BufferLen > 1 && Buffer[BufferLen-2] == '\r' && Buffer[BufferLen-1] == '\n') // reach the end of line
    {
      if (BufferLen > 2 && usart1_status != USART_RECEIVE_OVERFLW)
      {
        if (Buffer[0] == '+' && Buffer[1] == 'I')
          xMessageBufferSendFromISR(ReceiveQueueHandle,Buffer,BufferLen-2,NULL);
        else
          xMessageBufferSendFromISR(ESPRetQueueHandle,Buffer,BufferLen-2,NULL);
      }
      usart1_status = USART_RECEIVE_RUNNING;
      BufferLen = 0;
    }
    __HAL_UART_CLEAR_FLAG(&huart1, UART_FLAG_RXNE);
    return ;
  }
  if ((__HAL_UART_GET_FLAG(&huart1, UART_FLAG_IDLE) != RESET))
  {
    __HAL_UART_DISABLE_IT(&huart1,UART_IT_IDLE);
    if (BufferLen != 0 && usart1_status != USART_RECEIVE_OVERFLW)
    {
      if (BufferLen > 2 && Buffer[0] == '+' && Buffer[1] == 'I')
        xMessageBufferSendFromISR(ReceiveQueueHandle,Buffer,BufferLen,NULL);
      else if (!(BufferLen == 1 && Buffer[0] == ' '))
        itm_printf("Why?:#%.*s#\n",BufferLen,Buffer);
    }
    BufferLen = 0;
    usart1_status = USART_RECEIVE_PENDING;
  }

  // if ((__HAL_UART_GET_FLAG(&huart1, UART_FLAG_RXNE) != RESET))
  // {
  //   // itm_printf("-1");
  //   if (usart1_status == USART_RECEIVE_PENDING)
  //   {
  //     ReceivedPackagePos++;
  //     ReceivedPackagePos &= (RECEIVE_QUEUE_MAX - 1);
  //     ReceivedPackage[ReceivedPackagePos].len = 0;
  //     usart1_status = USART_RECEIVE_RUNNING;
  //     __HAL_UART_ENABLE_IT(&huart1, UART_IT_IDLE);
  //   }
  //   if (usart1_status != USART_RECEIVE_OVERFLW)
  //   {
  //     ReceivedPackage[ReceivedPackagePos].s[ReceivedPackage[ReceivedPackagePos].len++] = (uint8_t)(huart1.Instance->DR & (uint8_t)0x00FF);
  //     if (ReceivedPackage[ReceivedPackagePos].len == 1 && (ReceivedPackage[ReceivedPackagePos].s[0] == '\r' 
  //         || ReceivedPackage[ReceivedPackagePos].s[0] == '\n' || ReceivedPackage[ReceivedPackagePos].s[0] == ' '))
  //       ReceivedPackage[ReceivedPackagePos].len--;
  //     if (ReceivedPackage[ReceivedPackagePos].len == UART_BUF_SIZE)
  //       usart1_status = USART_RECEIVE_OVERFLW;
  //   }
  //   __HAL_UART_CLEAR_FLAG(&huart1, UART_FLAG_RXNE);
  //   return;
  // }
  // if ((__HAL_UART_GET_FLAG(&huart1, UART_FLAG_IDLE) != RESET))
  // {
  //   __HAL_UART_DISABLE_IT(&huart1,UART_IT_IDLE);
  //   if (usart1_status != USART_RECEIVE_OVERFLW)
  //   {
  //     // itm_printf("Again?");
  //     usart1_status = USART_RECEIVE_PENDING;
  //     if (ReceivedPackage[ReceivedPackagePos].len >=2 && ReceivedPackage[ReceivedPackagePos].s[0] == '+' && ReceivedPackage[ReceivedPackagePos].s[1] == 'I')
  //     {
  //       // xQueueSendFromISR(ReceiveQueueHandle,&ReceivedPackagePos,NULL);
  //       xStreamBufferSendFromISR(ReceiveQueueHandle,&ReceivedPackagePos,sizeof(uint8_t),NULL);
  //     }
  //     else
  //     {
  //       // xQueueSendFromISR(ESPRetQueueHandle,&ReceivedPackagePos,NULL);
  //       xStreamBufferSendFromISR(ESPRetQueueHandle,&ReceivedPackagePos,sizeof(uint8_t),NULL);
  //     }
  //     // itm_printf("No...\n");
  //   }
  //   else
  //     usart1_status = USART_RECEIVE_PENDING;
    
  //   return;
  // }
  /* USER CODE END USART1_IRQn 0 */
  HAL_UART_IRQHandler(&huart1);
  /* USER CODE BEGIN USART1_IRQn 1 */
  /* USER CODE END USART1_IRQn 1 */
}

/**
  * @brief This function handles TIM6 global interrupt, DAC1 and DAC2 underrun error interrupts.
  */
void TIM6_DAC_IRQHandler(void)
{
  /* USER CODE BEGIN TIM6_DAC_IRQn 0 */

  /* USER CODE END TIM6_DAC_IRQn 0 */
  HAL_TIM_IRQHandler(&htim6);
  /* USER CODE BEGIN TIM6_DAC_IRQn 1 */

  /* USER CODE END TIM6_DAC_IRQn 1 */
}

/**
  * @brief This function handles DMA2 stream0 global interrupt.
  */
void DMA2_Stream0_IRQHandler(void)
{
  /* USER CODE BEGIN DMA2_Stream0_IRQn 0 */

  /* USER CODE END DMA2_Stream0_IRQn 0 */
  HAL_DMA_IRQHandler(&hdma_adc1);
  /* USER CODE BEGIN DMA2_Stream0_IRQn 1 */

  /* USER CODE END DMA2_Stream0_IRQn 1 */
}

/**
  * @brief This function handles DMA2 stream3 global interrupt.
  */
void DMA2_Stream3_IRQHandler(void)
{
  /* USER CODE BEGIN DMA2_Stream3_IRQn 0 */

  /* USER CODE END DMA2_Stream3_IRQn 0 */
  HAL_DMA_IRQHandler(&hdma_spi1_tx);
  /* USER CODE BEGIN DMA2_Stream3_IRQn 1 */

  /* USER CODE END DMA2_Stream3_IRQn 1 */
}

/* USER CODE BEGIN 1 */

/* USER CODE END 1 */
/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
