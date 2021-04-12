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
#include "ESP8266.h"
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
extern uint32_t NetworkTick;
extern uint8_t ESP8266State, ESP8266Resend, ESP8266ForceClear;
extern uint8_t UDPState, NetworkState;

SemaphoreHandle_t ACKSemaphore[128];
uint8_t ACKTimes[128];
UDPRequest ACK[128];
uint32_t ACKcrc[128];

QueueHandle_t SendQueueHandle;
MessageBufferHandle_t RecieveQueueHandle;
MessageBufferHandle_t ESP8266RetQueueHandle;

/* USER CODE END Variables */
osThreadId CapADCTaskHandle;
osThreadId ChargerADCTaskHandle;
osThreadId MotorTaskHandle;
osThreadId NetworkSendTaskHandle;
osThreadId NetworkRecieveTHandle;
osThreadId ESP8266RetTaskHandle;
osThreadId NetworkCheckTasHandle;
osMutexId ACKQueueMutexHandle;
osMutexId ESP8266MutexHandle;
osMutexId UDPSendMutexHandle;
osSemaphoreId ESP8266RetHandle;

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
uint8_t SendUDPPackage(uint8_t op, uint8_t qos, char *data, size_t len);
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
void StartNetworkRecieveTask(void const * argument);
void StartESP8266RetTask(void const * argument);
void StartNetworkCheckTask(void const * argument);

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
  __HAL_UART_ENABLE_IT(&huart1, UART_IT_RXNE);
  idTail = (uint8_t)(HAL_RNG_GetRandomNumber(&hrng) >> (32-8+1));
  // dp.SCLK_GPIO_Port = DP_CLK_GPIO_Port;
  // dp.SCLK_Pin = DP_CLK_Pin;
  // dp.SDO_GPIO_Port = DP_DIO_GPIO_Port;
  // dp.SDO_Pin = DP_DIO_Pin;
  HAL_TIM_Base_Start_IT(&htim9);
  HAL_ADC_Start_DMA(&hadc1,&adc1_7_res,1);
  ACKSemaphoreInit();
  UDPState = UDP_NOT_READY;
  NetworkState = NETWORK_NOT_READY;
  // itm_printf("%lX\n",CRC32((uint8_t *)"asdfasdf",8));
  // itm_printf("%lX\n",CRC32((uint8_t *)"asdfasd",7));
  
  /* USER CODE END Init */
  /* Create the mutex(es) */
  /* definition and creation of ACKQueueMutex */
  osMutexDef(ACKQueueMutex);
  ACKQueueMutexHandle = osMutexCreate(osMutex(ACKQueueMutex));

  /* definition and creation of ESP8266Mutex */
  osMutexDef(ESP8266Mutex);
  ESP8266MutexHandle = osMutexCreate(osMutex(ESP8266Mutex));

  /* definition and creation of UDPSendMutex */
  osMutexDef(UDPSendMutex);
  UDPSendMutexHandle = osMutexCreate(osMutex(UDPSendMutex));

  /* USER CODE BEGIN RTOS_MUTEX */
  /* USER CODE END RTOS_MUTEX */

  /* Create the semaphores(s) */
  /* definition and creation of ESP8266Ret */
  osSemaphoreDef(ESP8266Ret);
  ESP8266RetHandle = osSemaphoreCreate(osSemaphore(ESP8266Ret), 1);

  /* USER CODE BEGIN RTOS_SEMAPHORES */
  xSemaphoreTake(ESP8266RetHandle,portMAX_DELAY);
  /* USER CODE END RTOS_SEMAPHORES */

  /* USER CODE BEGIN RTOS_TIMERS */
  /* start timers, add new ones, ... */
  /* USER CODE END RTOS_TIMERS */

  /* USER CODE BEGIN RTOS_QUEUES */
  /* add queues, ... */
  
  SendQueueHandle = xQueueCreate(16, sizeof (UDPRequest));

  RecieveQueueHandle = xMessageBufferCreate(UART_BUF_SIZE * 8);
  ESP8266RetQueueHandle = xMessageBufferCreate(UART_BUF_SIZE);
  
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
  osThreadDef(NetworkSendTask, StartNetworkSendTask, osPriorityLow, 0, 128);
  NetworkSendTaskHandle = osThreadCreate(osThread(NetworkSendTask), NULL);

  /* definition and creation of NetworkRecieveT */
  osThreadDef(NetworkRecieveT, StartNetworkRecieveTask, osPriorityBelowNormal, 0, 128);
  NetworkRecieveTHandle = osThreadCreate(osThread(NetworkRecieveT), NULL);

  /* definition and creation of ESP8266RetTask */
  osThreadDef(ESP8266RetTask, StartESP8266RetTask, osPriorityBelowNormal, 0, 128);
  ESP8266RetTaskHandle = osThreadCreate(osThread(ESP8266RetTask), NULL);

  /* definition and creation of NetworkCheckTas */
  osThreadDef(NetworkCheckTas, StartNetworkCheckTask, osPriorityNormal, 0, 128);
  NetworkCheckTasHandle = osThreadCreate(osThread(NetworkCheckTas), NULL);

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
  char s[2];
  HAL_ADC_Start_DMA(&hadc1,&adc1_7_res,1);
  HAL_TIM_Base_Start(&htim3);
  /* Infinite loop */
  for(uint8_t i=1;1;i++)
  {
    if (adc1_7_res > MAX_CAP_VOL * 1.0 * (1<<12) / 3.3 / 5)
      ChargerOff();
    if (adc1_7_res < MIN_CAP_VOL * 1.0 * (1<<12) / 3.3 / 5)
      ChargerOn();
    if (!i)
    {
      s[0] = (char)(adc1_7_res&0xFF);
      s[1] = (char)(adc1_7_res>>8);
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
void StartMotorTask(void const * argument)
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
  UDPRequest t;
  for(;;)
  {
    xQueueReceive(SendQueueHandle,&t,portMAX_DELAY);
    // if (t.id!=0)
    //   itm_printf("%d: %p, %d\n",t.id,t.data,ACKTimes[t.id]);
    if (t.id)
      xTaskCreate((void *)StartSingleACKTask, NULL, 128, (void *)&t.id, osPriorityNormal, NULL);
    if (xSemaphoreTake(ESP8266MutexHandle,pdMS_TO_TICKS(700)) == pdPASS)
    {
      ESP8266Send(t.data, t.len);
      xSemaphoreGive(ESP8266MutexHandle);
    }
    if (!t.id)
      vPortFree(t.data);
    osDelay(5);
  }
  /* USER CODE END StartNetworkSendTask */
}

/* USER CODE BEGIN Header_StartNetworkRecieveTask */
/**
* @brief Function implementing the NetworkRecieveT thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_StartNetworkRecieveTask */
void StartNetworkRecieveTask(void const * argument)
{
  /* USER CODE BEGIN StartNetworkRecieveTask */
  /* Infinite loop */
  uint8_t op, qos, len, prt, id;
  size_t rlen,elen;
  uint8_t *raw, *p, *praw;
  praw = raw = (uint8_t *)pvPortMalloc(UART_BUF_SIZE+1);
  uint16_t header;
  uint32_t crc;
  for(;;)
  {
    // xQueueReceive(RecieveQueueHandle,&pos,portMAX_DELAY);
    // xStreamBufferReceive(RecieveQueueHandle,&pos,sizeof(uint8_t),portMAX_DELAY);
    raw = praw;
    rlen = xMessageBufferReceive(RecieveQueueHandle,raw,UART_BUF_SIZE,portMAX_DELAY);
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
    if (op == 0x7) // recieve ACK
    {
      if (rlen != 6) // ACK len error, drop
        continue;
      id = header >> 9;
      crc = *((uint32_t *)(raw+2));
      // itm_printf("crc:%lX\n",crc);
      if (ACK[id].id == 0 || crc != ACKcrc[id]) // ACK no exist, drop
        continue;
      if (header >> 3 & 1) // ACK request resend NOT WORKING
        xQueueSend(SendQueueHandle,&ACK[id],pdMS_TO_TICKS(200)); // recieve resend ACK
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
      // itm_printf("%d %lX\n",id,crc);
      SendACKPackage(id,0,crc);
      raw += 1;
    }
    raw += 2;
    if (op == 0x0)
      itm_printf("Recieve: #%s#\n",raw);
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
  /* USER CODE END StartNetworkRecieveTask */
}

/* USER CODE BEGIN Header_StartESP8266RetTask */
/**
* @brief Function implementing the ESP8266RetTask thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_StartESP8266RetTask */
void StartESP8266RetTask(void const * argument)
{
  /* USER CODE BEGIN StartESP8266RetTask */
  /* Infinite loop */
  size_t rlen, st;
  uint8_t *raw, *praw;
  praw = raw = (uint8_t *)pvPortMalloc(UART_BUF_SIZE+1);
  uint8_t state = ESP_NON;
  for(;;)
  {
    raw = praw;
    rlen = xMessageBufferReceive(ESP8266RetQueueHandle,raw,UART_BUF_SIZE,portMAX_DELAY);
    if (rlen == 0)
      itm_printf("Weird, xMBR report buffer too small\n");
    raw[rlen]='\0';
    // for (uint8_t i = 0;i<rlen;i++)
    //   itm_printf("%02x",raw[i]);
    // itm_printf("\n%d:@%s@\n",rlen,raw);
    for (st=0;st < rlen && (raw[st]=='\n' || raw[st]=='\r' || raw[st]==' ');st++);
    if (rlen-st >= 15 && strstr((char *)(raw+st), "WIFI DISCONNECT") != NULL)
    {
      // for now, hold mutex will make ACK array overflow
      // fixed
      NetworkState = NETWORK_NOT_READY;
      itm_printf("Wifi lost...\n");
    }
    else if (rlen-st >= 14 && strstr((char *)(raw+st), "WIFI CONNECTED") != NULL)
    {
      itm_printf("Wifi connected...\n");
    }
    else if (rlen-st >= 11 && strstr((char *)(raw+st), "WIFI GOT IP") != NULL)
    {
      NetworkState = NETWORK_READY;
      itm_printf("Got wifi IP...\n");
    }
    else if (rlen-st >= 7 && strstr((char *)(raw+st), "+CWJAP:") != NULL)
    {
      state = ESP_CWJ;
      // itm_printf("state:ESP_CWJ\n");
    }
    else if (rlen-st >= 4 && strstr((char *)(raw+st), "FAIL") != NULL)
    {
      if (state == ESP_CWJ)
      {
        NetworkState = NETWORK_NOT_READY;
        itm_printf("Failed to connect to wifi...\n");
        if (ESP8266State == ESP8266_STATE_CNNT)
          xSemaphoreGive(ESP8266RetHandle);
        else
          itm_printf("CWJAP FAIL:state error:%d\n",ESP8266State);
        state = ESP_NON;
        // itm_printf("state:ESP_NON\n");
      }
      else
        itm_printf("FAIL:missing ESP_CWJ:%d\n",state);
    }
    else if (rlen-st >= 5 && strstr((char *)(raw+st), "no ip") != NULL)
    {
      UDPState = UDP_NOT_READY;
      itm_printf("UDP connect failed...\n");
      if (ESP8266State == ESP8266_STATE_OPEN)
        xSemaphoreGive(ESP8266RetHandle);
      else
        itm_printf("no ip:state error:%d\n",ESP8266State);
    }
    else if (rlen-st >= 17 && strstr((char *)(raw+st), "link is not valid") != NULL)
    {
      UDPState = UDP_NOT_READY;
      itm_printf("UDP send failed...\n");
      if (ESP8266State == ESP8266_STATE_HEAD)
        xSemaphoreGive(ESP8266RetHandle);
      else
        itm_printf("link is not valid:state error:%d\n",ESP8266State);
    }
    else if (rlen-st >= 2 && strstr((char *)(raw+st), "OK") != NULL)
    {
      if (ESP8266State == ESP8266_STATE_INIT)
        xSemaphoreGive(ESP8266RetHandle);
      else if (ESP8266State == ESP8266_STATE_CNNT)
      {
        NetworkState = NETWORK_READY;
        xSemaphoreGive(ESP8266RetHandle);
      }
      else
        state = ESP_GOK;
    }
    else if (rlen-st >= 2 && strstr((char *)(raw+st), "> ") != NULL)
    {
      if (state == ESP_GOK)
      {
        if (ESP8266State == ESP8266_STATE_HEAD)
          xSemaphoreGive(ESP8266RetHandle);
        else
        {
          itm_printf("OK > :state error:%d\n",ESP8266State);
          ESP8266ForceClear = 1;
        }
        state = ESP_NON;
      }
      else
      {
        itm_printf("> :missing ESP_GOK:%d\n",state);
        ESP8266ForceClear = 1;
      }
    }
    else if (rlen-st >= 17 && strstr((char *)(raw+st), "ALREADY CONNECTED") != NULL)
    {
      if (ESP8266State == ESP8266_STATE_OPEN)
      {
        UDPState = UDP_READY;
        xSemaphoreGive(ESP8266RetHandle);
      }
      else
        itm_printf("ALREADY CONNECTED:state error:%d\n",ESP8266State);
    }
    else if (rlen-st >= 7 && strstr((char *)(raw+st), "CONNECT") != NULL)
    {
      if (ESP8266State == ESP8266_STATE_OPEN)
      {
        UDPState = UDP_READY;
        xSemaphoreGive(ESP8266RetHandle);
      }
      else
        itm_printf("CONNECT:state error:%d\n",ESP8266State);
    }
    else if (rlen-st >= 4 && strstr((char *)(raw+st), "Recv") != NULL)
    {
      if (ESP8266State == ESP8266_STATE_SEND)
        xSemaphoreGive(ESP8266RetHandle);
      else
        itm_printf("Recv:state error:%d\n",ESP8266State);
    }
    else if (((rlen-st >= 9 && strstr((char *)(raw+st), "busy p...") != NULL) || (rlen-st >= 9 && strstr((char *)(raw+st), "busy s...") != NULL)) && ESP8266State != ESP8266_STATE_IDLE)
    {
      ESP8266Resend = 1;
      xSemaphoreGive(ESP8266RetHandle);
    }
    osDelay(1);
  }
  /* USER CODE END StartESP8266RetTask */
}

/* USER CODE BEGIN Header_StartNetworkCheckTask */
/**
* @brief Function implementing the NetworkCheckTas thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_StartNetworkCheckTask */
void StartNetworkCheckTask(void const * argument)
{
  /* USER CODE BEGIN StartNetworkCheckTask */
  osMutexWait(ESP8266MutexHandle, osWaitForever);
  ESP8266Init();
  osMutexRelease(ESP8266MutexHandle);
  /* Infinite loop */
  for(;;)
  {
    if (NetworkState != NETWORK_READY)
    {
      osMutexWait(ESP8266MutexHandle, osWaitForever);
      ESP8266Connect();
      osMutexRelease(ESP8266MutexHandle);
    }
    if (NetworkState == NETWORK_READY && UDPState != UDP_READY)
    {
      osMutexWait(ESP8266MutexHandle, osWaitForever);
      ESP8266Open();
      osMutexRelease(ESP8266MutexHandle);
    }
    osDelay(5000);
  }
  /* USER CODE END StartNetworkCheckTask */
}

/* Private application code --------------------------------------------------*/
/* USER CODE BEGIN Application */

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
    // itm_printf("Got %d ACK!\n",id);
    FreeACK(id);
  }
  vTaskDelete(NULL);
}

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

uint8_t SendUDPPackage(uint8_t op, uint8_t qos, char *data, size_t len)
{
  osMutexWait(UDPSendMutexHandle,osWaitForever);
  if (data!=NULL && len == 0xFFFF)
    len = strlen(data);
  uint8_t *raw;
  UDPRequest t;
  if (op>=(1<<3) || len>=(1<<6) || qos>=(1<<1))
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
