/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * File Name          : freertos.c
  * Description        : Code for freertos applications
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
#include "FreeRTOS.h"
#include "task.h"
#include "main.h"
#include "cmsis_os.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
// #include "printf-stdarg.h"
#include "retarget.h"
#include <stdio.h>
#include <string.h>
#include "adc.h"
#include "dma.h"
#include "spi.h"
#include "tim.h"
#include "crc.h"
#include "usart.h"
#include "gpio.h"
#include "rng.h"
// #include "TM1637.h"
#include "ssd1306.h"
#include "ssd1306_tests.h"
#include "ESP.h"
#include "stream_buffer.h"
#include "message_buffer.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define ACK_MAX_RETRY 3
#define ACK_OVERFLOW 0xFF
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
/* USER CODE BEGIN Variables */
uint32_t adc1_7_res;
// TM1637 dp;
uint8_t idTail;

SemaphoreHandle_t ACKSemaphore[128];
uint8_t ACKTimes[128];
UDPRequest ACK[128];
uint32_t ACKcrc[128];

QueueHandle_t SendQueueHandle;
MessageBufferHandle_t ReceiveQueueHandle;

/* USER CODE END Variables */
osThreadId CapADCTaskHandle;
osThreadId ChargerADCTaskHandle;
osThreadId MotorTaskHandle;
osThreadId NetworkSendTaskHandle;
osThreadId NetworkReceiveTHandle;
osMutexId ACKQueueMutexHandle;
osMutexId UDPSendMutexHandle;

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN FunctionPrototypes */
uint32_t CRC32(uint8_t *data,size_t len);
void ChargerOff();
void ChargerOn();
void MotorStart();
void MotorStop();
void MotorSpeed();
uint8_t BitCount(uint8_t *data, size_t len);
uint8_t SendACKPackage(uint8_t id, uint8_t resend, uint32_t crc);
uint8_t SendUDPPackage(uint8_t op, uint8_t qos, uint8_t *data, size_t len);
uint8_t SendAMGData(const AMGData * data,int qos);
inline uint16_t HammingUnpack(uint16_t v);
inline uint16_t HammingPack(uint16_t v);
inline uint8_t GetACKid();
void FreeACK(uint8_t id);
void ACKSemaphoreInit();
void StartSingleACKTask(void const * argument);
/* USER CODE END FunctionPrototypes */

void StartCapADCTask(void const * argument);
void StartChargerADCTask(void const * argument);
void StartMotorTask(void const * argument);
void StartNetworkSendTask(void const * argument);
void StartNetworkReceiveTask(void const * argument);

void MX_FREERTOS_Init(void); /* (MISRA C 2004 rule 8.1) */

/* GetIdleTaskMemory prototype (linked to static allocation support) */
void vApplicationGetIdleTaskMemory( StaticTask_t **ppxIdleTaskTCBBuffer, StackType_t **ppxIdleTaskStackBuffer, uint32_t *pulIdleTaskStackSize );

/* USER CODE BEGIN GET_IDLE_TASK_MEMORY */
static StaticTask_t xIdleTaskTCBBuffer;
static StackType_t xIdleStack[configMINIMAL_STACK_SIZE];

void vApplicationGetIdleTaskMemory( StaticTask_t **ppxIdleTaskTCBBuffer, StackType_t **ppxIdleTaskStackBuffer, uint32_t *pulIdleTaskStackSize )
{
  *ppxIdleTaskTCBBuffer = &xIdleTaskTCBBuffer;
  *ppxIdleTaskStackBuffer = &xIdleStack[0];
  *pulIdleTaskStackSize = configMINIMAL_STACK_SIZE;
  /* place for user code */
}
/* USER CODE END GET_IDLE_TASK_MEMORY */

/**
  * @brief  FreeRTOS initialization
  * @param  None
  * @retval None
  */
void MX_FREERTOS_Init(void) {
  /* USER CODE BEGIN Init */
  // ACKQueueTail=ACKQueueHead=0;
  RetargetInit();
  idTail = (uint8_t)(HAL_RNG_GetRandomNumber(&hrng) >> (32-8+1));
  // dp.SCLK_GPIO_Port = DP_CLK_GPIO_Port;
  // dp.SCLK_Pin = DP_CLK_Pin;
  // dp.SDO_GPIO_Port = DP_DIO_GPIO_Port;
  // dp.SDO_Pin = DP_DIO_Pin;
  HAL_TIM_Base_Start_IT(&htim9);
  HAL_ADC_Start_DMA(&hadc1,&adc1_7_res,1);
  ACKSemaphoreInit();
  // itm_printf("%lX\n",CRC32((uint8_t *)"asdfasdf",8));
  // itm_printf("%lX\n",CRC32((uint8_t *)"asdfasd",7));
  
  /* USER CODE END Init */
  /* Create the mutex(es) */
  /* definition and creation of ACKQueueMutex */
  osMutexDef(ACKQueueMutex);
  ACKQueueMutexHandle = osMutexCreate(osMutex(ACKQueueMutex));

  /* definition and creation of UDPSendMutex */
  osMutexDef(UDPSendMutex);
  UDPSendMutexHandle = osMutexCreate(osMutex(UDPSendMutex));

  /* USER CODE BEGIN RTOS_MUTEX */
  /* USER CODE END RTOS_MUTEX */

  /* USER CODE BEGIN RTOS_SEMAPHORES */
  /* USER CODE END RTOS_SEMAPHORES */

  /* USER CODE BEGIN RTOS_TIMERS */
  /* start timers, add new ones, ... */
  /* USER CODE END RTOS_TIMERS */

  /* USER CODE BEGIN RTOS_QUEUES */
  /* add queues, ... */
  
  SendQueueHandle = xQueueCreate(16, sizeof (UDPRequest));

  ReceiveQueueHandle = xMessageBufferCreate(UART_BUF_SIZE * 8);
  
  // HAL_UART_Receive_IT(&huart1,&recv_byte,1);
  itm_printf("InitHere!!!\n");
  /* USER CODE END RTOS_QUEUES */

  /* Create the thread(s) */
  /* definition and creation of CapADCTask */
  osThreadDef(CapADCTask, StartCapADCTask, osPriorityHigh, 0, 128);
  CapADCTaskHandle = osThreadCreate(osThread(CapADCTask), NULL);

  /* definition and creation of ChargerADCTask */
  osThreadDef(ChargerADCTask, StartChargerADCTask, osPriorityNormal, 0, 128);
  ChargerADCTaskHandle = osThreadCreate(osThread(ChargerADCTask), NULL);

  /* definition and creation of MotorTask */
  osThreadDef(MotorTask, StartMotorTask, osPriorityAboveNormal, 0, 128);
  MotorTaskHandle = osThreadCreate(osThread(MotorTask), NULL);

  /* definition and creation of NetworkSendTask */
  osThreadDef(NetworkSendTask, StartNetworkSendTask, osPriorityHigh, 0, 128);
  NetworkSendTaskHandle = osThreadCreate(osThread(NetworkSendTask), NULL);

  /* definition and creation of NetworkReceiveT */
  osThreadDef(NetworkReceiveT, StartNetworkReceiveTask, osPriorityBelowNormal, 0, 128);
  NetworkReceiveTHandle = osThreadCreate(osThread(NetworkReceiveT), NULL);

  /* USER CODE BEGIN RTOS_THREADS */
  /* add threads, ... */
  /* USER CODE END RTOS_THREADS */

}

/* USER CODE BEGIN Header_StartCapADCTask */
/**
  * @brief  Function implementing the CapADCTask thread.
  * @param  argument: Not used
  * @retval None
  */
/* USER CODE END Header_StartCapADCTask */
void StartCapADCTask(void const * argument)
{
  /* USER CODE BEGIN StartCapADCTask */
  uint8_t s[2];
  HAL_ADC_Start_DMA(&hadc1,&adc1_7_res,1);
  HAL_TIM_Base_Start(&htim3);
  /* Infinite loop */
  
  //speed test
  // uint8_t *raw = (uint8_t *)pvPortMalloc(500);
  // memset(raw,'a',500);
  // for (;;)
  // {
  //   SendUDPPackage(0,0,raw,500);
  //   // osDelay(40);
  // }
  //speed test

  for(uint8_t i=1;1;i++)
  {
    if (adc1_7_res > MAX_CAP_VOL * 1.0 * (1<<12) / 3.3 / 5)
      ChargerOff();
    if (adc1_7_res < MIN_CAP_VOL * 1.0 * (1<<12) / 3.3 / 5)
      ChargerOn();
    if (!i)
    {
      s[0] = (uint8_t)(adc1_7_res&0xFF);
      s[1] = (uint8_t)(adc1_7_res>>8);
      SendUDPPackage(6,0,s,2);
    }
    osDelay(5);
  }
  /* USER CODE END StartCapADCTask */
}

/* USER CODE BEGIN Header_StartChargerADCTask */
/**
* @brief Function implementing the ChargerADCTask thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_StartChargerADCTask */
void StartChargerADCTask(void const * argument)
{
  /* USER CODE BEGIN StartChargerADCTask */
  /* Infinite loop */
  for(;;)
  {
    osDelay(5);
  }
  /* USER CODE END StartChargerADCTask */
}

/* USER CODE BEGIN Header_StartMotorTask */
/**
* @brief Function implementing the MotorTask thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_StartMotorTask */
__weak void StartMotorTask(void const * argument)
{
  /* USER CODE BEGIN StartMotorTask */
  /* Infinite loop */
  for(;;)
  {
    osDelay(1);
  }
  /* USER CODE END StartMotorTask */
}

/* USER CODE BEGIN Header_StartNetworkSendTask */
/**
* @brief Function that manage the sending of all normal UDP package
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_StartNetworkSendTask */
void StartNetworkSendTask(void const * argument)
{
  /* USER CODE BEGIN StartNetworkSendTask */
  /* Infinite loop */
  ESPInit();
  __HAL_UART_ENABLE_IT(&huart1, UART_IT_RXNE);
  UDPRequest t;
  for(;;)
  {
    xQueueReceive(SendQueueHandle,&t,portMAX_DELAY);
    // if (t.id!=0)
    //   itm_printf("%d: %p, %d\n",t.id,t.data,ACKTimes[t.id]);
    if (t.id)
      xTaskCreate((void *)StartSingleACKTask, NULL, 128, (void *)&t.id, osPriorityNormal, NULL);
    ESPSend(t.data, t.len);
    if (!t.id)
      vPortFree(t.data);
    osDelay(5);
  }
  /* USER CODE END StartNetworkSendTask */
}

/* USER CODE BEGIN Header_StartNetworkReceiveTask */
/**
* @brief Function implementing the NetworkReceiveT thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_StartNetworkReceiveTask */
void StartNetworkReceiveTask(void const * argument)
{
  /* USER CODE BEGIN StartNetworkReceiveTask */
  /* Infinite loop */
  uint8_t op, qos, prt, id;
  size_t rlen,elen, len;
  uint8_t *raw, *p, *praw;
  praw = raw = (uint8_t *)pvPortMalloc(UART_BUF_SIZE+1);
  uint16_t header;
  uint32_t crc;
  for(;;)
  {
    // xQueueReceive(ReceiveQueueHandle,&pos,portMAX_DELAY);
    // xStreamBufferReceive(ReceiveQueueHandle,&pos,sizeof(uint8_t),portMAX_DELAY);
    raw = praw;
    rlen = xMessageBufferReceive(ReceiveQueueHandle,raw,UART_BUF_SIZE,portMAX_DELAY);
    if (rlen == 0)
      itm_printf("Weird, xMBR report buffer too small\n");
    raw[rlen]='\0';
    p = (uint8_t *)strchr((char *)raw, ':');
    // itm_printf("%d:#%s#\n",rlen,raw);
    // for (uint8_t i = 0;i<rlen;i++)
    //   itm_printf("%02x",raw[i]);
    // itm_printf("\n");
    if (p == NULL) // unknown uart package type, drop
      continue;
    elen = 0;
    for (uint8_t *i=raw+5;i<p;i++)
      elen = elen*10 + (*i) - '0';
    if (elen != rlen - (p + 1 - raw)) // uart package len error, drop
      continue;
    raw = p + 1;
    rlen = elen;
    header = HammingUnpack(*((uint16_t *)raw));
    // for now, we don't support ACK request resend
    if (__builtin_popcount(header) & 1) // header check failed, drop package
      continue;
    op  = header >> 5 & 0x07;
    if (op == 0x7) // receive ACK
    {
      if (rlen != 6) // ACK len error, drop
        continue;
      id = header >> 9;
      crc = *((uint32_t *)(raw+2));
      // itm_printf("crc:%lX\n",crc);
      if (id == 0)
        continue;
      if (ACK[id].id == 0 || crc != ACKcrc[id]) // ACK no exist, drop
        continue;
      if (header >> 3 & 1) // ACK request resend NOT WORKING
        xQueueSend(SendQueueHandle,&ACK[id],pdMS_TO_TICKS(200)); // receive resend ACK
      xSemaphoreGive(ACKSemaphore[id]);
      continue;
    }
    len = (header >> 9 & 0x3F) << 3;
    qos = header >> 3 & 0x01;
    prt = header >> 15;
    // itm_printf("%d:#%s#\n",rlen,raw);
    // itm_printf("rlen:%d, len:%d, op:%d, qos:%d, prt:%d\n",rlen,len,op,qos,prt);
    if (len + qos + 2 != rlen) // len error, drop
      continue;
    if (len !=0 && prt != BitCount(raw+2+qos, len))
      continue;
    if (qos)
    {
      crc = CRC32(raw+3,len);
      id = raw[2];
      if (__builtin_popcount(id) & 0x1) // id parity check failed, drop
        continue;
      id &= 0x7F;
      if (id == 0)
        continue;
      // itm_printf("%d %lX\n",id,crc);
      SendACKPackage(id,0,crc);
      raw += 1;
    }
    raw += 2;
    if (op == 0x0)
      itm_printf("Receive: #%s#\n",raw);
    else if (op == 0x1)
      MotorStop();
    else if (op == 0x2)
      MotorStart();
    else if (op == 0x3)
      ChargerOff();
    else if (op == 0x4)
      ChargerOn();
    else if (op == 0x5)
    {
      if (len != 2) // op 0x5 len error, drop
        continue;
      MotorSpeed(*((uint16_t *)raw));
    }
    else if (op == 0x6)
    {
      // PING
    }
    osDelay(1);
  }
  /* USER CODE END StartNetworkReceiveTask */
}

/* Private application code --------------------------------------------------*/
/* USER CODE BEGIN Application */

void ChargerOn()
{
  if (adc1_7_res <= MAX_CAP_VOL * 1.0 * (1<<12) / 3.3 / 5)
    HAL_GPIO_WritePin(ChargerCtrl_GPIO_Port,ChargerCtrl_Pin,GPIO_PIN_RESET);
}
void ChargerOff()
{
  
  if (adc1_7_res >= MIN_CAP_VOL * 1.0 * (1<<12) / 3.3 / 5)
    HAL_GPIO_WritePin(ChargerCtrl_GPIO_Port,ChargerCtrl_Pin,GPIO_PIN_SET);
}
void MotorStart() {}
void MotorStop() {}
void MotorSpeed() {}

uint8_t SendAMGData(const AMGData * data,int qos)
{
  uint8_t *raw = pvPortMalloc(12 * 64 / 8);
  uint8_t *p = raw;
  uint16_t a,b;
  uint8_t ret;
  for (int i=0,j=1;i<64;i+=2,j+=2)
  {
    a = (data->data[i] & ((1<<12) - 1));
    b = (data->data[j] & ((1<<12) - 1));
    *(p++) = a & ((1<<8) - 1);
    *(p++) = (a >> 8) || b & ((1<<4) - 1);
    *(p++) = b >> 4;
  }
  ret = SendUDPPackage(1,0,raw,12 * 64 / 8);
  vPortFree(raw);
  return ret;
}

uint32_t CRC32(uint8_t *data,size_t len)
{
  if (len==0)
    return 0;
  size_t rlen = (len-1)/4+1;
  uint32_t *raw = (uint32_t *)pvPortMalloc(rlen*4);
  raw[rlen-1] = 0;
  memcpy(raw,data,len);
  for (int i=0;i<rlen;i++)
    raw[i]= __RBIT(raw[i]);
  uint32_t ret = HAL_CRC_Calculate(&hcrc,raw,rlen);
  vPortFree(raw);
  return __RBIT(ret)^0xFFFFFFFF;
}

void FreeACK(uint8_t id)
{
  vPortFree(ACK[id].data);
  ACKTimes[id] = 0;
  ACK[id].len = 0;
  ACK[id].id = 0;
}

void StartSingleACKTask(void const * argument)
{
  uint8_t id = *((uint8_t *)argument);
  // itm_printf("%d %d %p\n",ACK[id].id,ACK[id].len,ACK[id].data);
  if (xSemaphoreTake(ACKSemaphore[id],pdMS_TO_TICKS(1500)) != pdPASS)
  {
    if (ACKTimes[id]==ACK_MAX_RETRY)
    {
      itm_printf("%d reach ACK_MAX_RETRY time..\n",id);
      FreeACK(id);
    }
    else if (ACKTimes[id]==ACK_OVERFLOW)
    {
      itm_printf("ACK array overflow!!(%d cleaned)\n",id);
      FreeACK(id);
    }
    else
    {
      ACKTimes[id]++;
      if (xQueueSend(SendQueueHandle,&(ACK[id]),pdMS_TO_TICKS(200)) != pdPASS)
        FreeACK(id);
    }
  }
  else
  {
    itm_printf("Got %d ACK!\n",id);
    FreeACK(id);
  }
  vTaskDelete(NULL);
}

uint8_t BitCount(uint8_t *data, size_t len)
{
  uint8_t ret = 0;
  while (len--)
    ret ^= __builtin_popcount(data[len]) & 1;
  return ret;
}

uint8_t SendACKPackage(uint8_t id, uint8_t resend, uint32_t crc)
{
  if (id>=(1<<7))
    return NETWORK_SEND_FAILED;
  uint16_t header = (resend<<3) | (0x7 << 5) | (id << 9);
  UDPRequest t;
  header = HammingPack(header);
  header = header | (__builtin_popcount(header) & 1);
  uint8_t *raw = (uint8_t *)pvPortMalloc(6);
  memcpy(raw,&header,2);
  memcpy(raw+2,&crc,4);
  t.data = raw;
  t.len = 6;
  t.id = 0;
  if (xQueueSend(SendQueueHandle,&t,pdMS_TO_TICKS(100))!=pdPASS)
  {
    vPortFree(raw);
    return NETWORK_SEND_FAILED;
  }
  return NETWORK_SEND_OK;
}

uint8_t SendUDPPackage(uint8_t op, uint8_t qos, uint8_t *data, size_t len)
{
  osMutexWait(UDPSendMutexHandle,osWaitForever);
  if (data!=NULL && len == 0xFFFF)
    len = strlen((char *)data);
  uint8_t *raw;
  UDPRequest t;
  if (op>=(1<<3) || len>=(1<<9) || qos>=(1<<1))
    return NETWORK_SEND_FAILED;
  size_t plen = (((len & 0x7) != 0) + (len >> 3)) << 3;
  uint16_t header = (qos << 3) | (op << 5) | ((plen >> 3) << 9) | ((BitCount((uint8_t *)data, len)) << 15);
  header = HammingPack(header);
  header = header | (__builtin_popcount(header) & 1);
  
  if (qos == QOS_ACK)
  {
    uint8_t id = GetACKid();
    if (id == 0)
    {
      itm_printf("id has ran out\n");
      osMutexRelease(UDPSendMutexHandle);
      return NETWORK_SEND_FAILED;
    }
    // itm_printf("%d\n",xPortGetFreeHeapSize());
    raw = (uint8_t *)pvPortMalloc(plen + 3);
    // itm_printf("%p:%d %d\n",raw,len,plen);
    memcpy(raw,(uint8_t *)(&header),2);
    raw[2] = id | ((__builtin_popcount(id) & 1) << 7);
    if (len != 0)
      memcpy(raw+3,(uint8_t *)data,len);
    if (len & 0x7)
      memset(raw+3+len,0,plen-len);
    len = plen + 3;
    t.id = id;
    ACK[id].data = raw;
    ACK[id].len = len;
    ACK[id].id = id;
    ACKcrc[id] = CRC32(raw+3, plen);
  }
  else
  {
    // itm_printf("%d\n",xPortGetFreeHeapSize());
    raw = (uint8_t *)pvPortMalloc(plen + 2);
    // itm_printf("%p\n",raw);
    memcpy(raw,(uint8_t *)(&header),2);
    if (len != 0)
      memcpy(raw+2,(uint8_t *)data,len);
    if (len & 0x7)
      memset(raw+2+len,0,plen-len);
    len = plen + 2;
    t.id = 0;
  }
  t.data = raw;
  t.len = len;
  if (xQueueSend(SendQueueHandle,&t,pdMS_TO_TICKS(100))!=pdPASS)
  {
    vPortFree(raw);
    osMutexRelease(UDPSendMutexHandle);
    return NETWORK_SEND_FAILED;
  }
  osMutexRelease(UDPSendMutexHandle);
  return NETWORK_SEND_OK;
}

uint16_t HammingUnpack(uint16_t v)
{
  uint16_t p = 0;
  for (uint8_t i=0;i<16;i++)
    p ^= (1<<i) & v ? i : 0;
  return p ? v^(1<<p) : v;
  /*check: __builtin_popcount(ret) != 0 --> more than 1*/
}

uint16_t HammingPack(uint16_t v)
{
  v |= (((v >> 3) ^ (v >> 5) ^ (v >> 7) ^ (v >> 9) ^ (v >> 11) ^ (v >> 13) ^ (v >> 15)) & 1) << 1;
  v |= (((v >> 3) ^ (v >> 6) ^ (v >> 7) ^ (v >> 10) ^ (v >> 11) ^ (v >> 14) ^ (v >> 15)) & 1) << 2;
  v |= (((v >> 5) ^ (v >> 6) ^ (v >> 7) ^ (v >> 12) ^ (v >> 13) ^ (v >> 14) ^ (v >> 15)) & 1) << 4;
  v |= (((v >> 9) ^ (v >> 10) ^ (v >> 11) ^ (v >> 12) ^ (v >> 13) ^ (v >> 14) ^ (v >> 15)) & 1) << 8;
  return v;
}

uint8_t GetACKid()
{
  uint8_t i=idTail | 0x80;
  for (;idTail != i && ((i & 0x7F) == 0 || ACK[i&0x7F].id != 0);i++);
  idTail = (i + 1) & 0x7F;
  return ACK[i ^= 0x80].id ? 0 : i;
}

void ACKSemaphoreInit()
{
  for (uint8_t i=0;i<(1<<7);i++)
    ACKSemaphore[i]=xSemaphoreCreateBinary();
}

/* USER CODE END Application */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
