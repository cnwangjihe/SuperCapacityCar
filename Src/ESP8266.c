#include "ESP8266.h"
#include <stdio.h>
#include <string.h>
#include "usart.h"
#include "FreeRTOS.h"
#include "task.h"
#include "cmsis_os.h"
#include "stream_buffer.h"

extern osSemaphoreId ESPRetHandle;
uint8_t UDPState, NetworkState;
uint8_t ESPState, ESPRes, ESPForceClear;
uint8_t ESPSPIState;
StreamBufferHandle_t SPISendBuffer,SPIReceiveBuffer;

void ESPClear()
{
    if (!ESPForceClear)
        return ;
    itm_printf("Force clear triggered!\n");
    HAL_UART_Transmit(&huart1,(uint8_t *)"\r\n\r\n\r\n\r\n",8,1000);
    // itm_printf("@@%.*s@@",sizeof(ESP_UDP_MAXL)-1,ESP_UDP_MAXL);
    HAL_UART_Transmit(&huart1,(uint8_t *)ESP_UDP_MAXL,sizeof(ESP_UDP_MAXL)-1,1000);
    HAL_UART_Transmit(&huart1,(uint8_t *)"\r\n\r\n\r\n\r\n",8,1000);
    ESPForceClear = 0;
}

void ESPConnect()
{
    ESPClear();
    ESPState = ESP_STATE_CNNT;
    do{
        HAL_UART_Transmit(&huart1,(uint8_t *)ESP_WIFI_CON,sizeof(ESP_WIFI_CON)-1,1000);
        ITMassert(xSemaphoreTake(ESPRetHandle,pdMS_TO_TICKS(10000)),"Connect Semaphore Timeout\n");
    }while (ESPRes == ESP_RES_RSNT);
    ESPState = ESP_STATE_IDLE;
}

void ESPInit()
{
    ESPForceClear = 1;
    ESPClear();
    ESPState = ESP_STATE_INIT;
    do{
        HAL_UART_Transmit(&huart1,(uint8_t *)ESP_WIFI_DIS,sizeof(ESP_WIFI_DIS)-1,1000);
        ITMassert(xSemaphoreTake(ESPRetHandle,pdMS_TO_TICKS(500)),"Init Semaphore Timeout\n");
    }while (ESPRes == ESP_RES_RSNT);
    ESPState = ESP_STATE_IDLE;
}

void ESPClose()
{
    ESPClear();
    ESPState = ESP_STATE_CLOS;
    do{
        ESPRes = ESP_RES_NONE;
        HAL_UART_Transmit(&huart1,(uint8_t *)ESP_UDP_CLOS,sizeof(ESP_UDP_CLOS)-1,1000);
        ITMassert(xSemaphoreTake(ESPRetHandle,pdMS_TO_TICKS(500)),"Close Semaphore Timeout\n");
    }while (ESPRes == ESP_RES_RSNT);
    ESPState = ESP_STATE_IDLE;
}

void ESPOpen()
{
    ESPClear();
    ESPState = ESP_STATE_OPEN;
    do{
        ESPRes = ESP_RES_NONE;
        HAL_UART_Transmit(&huart1,(uint8_t *)ESP_UDP_OPEN,sizeof(ESP_UDP_OPEN)-1,1000);
        ITMassert(xSemaphoreTake(ESPRetHandle,pdMS_TO_TICKS(500)),"Open Semaphore Timeout\n");
        if (ESPRes == ESP_RES_CLOS)
            ESPClose();
    }while (ESPRes == ESP_RES_RSNT);
    ESPState = ESP_STATE_IDLE;
}

uint8_t ESPSendHead(size_t len)
{
    uint8_t ret;
    ESPClear();
    char tmp[8];
    sprintf(tmp,"%d\r\n",len);
    ESPState = ESP_STATE_HEAD;
    do{
        ESPRes = ESP_RES_NONE;
        HAL_UART_Transmit(&huart1,(uint8_t *)ESP_UDP_HEAD,sizeof(ESP_UDP_HEAD)-1,1000);
        HAL_UART_Transmit(&huart1,(uint8_t *)tmp,strlen(tmp),2000);
        ret = xSemaphoreTake(ESPRetHandle,pdMS_TO_TICKS(2000));
        ITMassert(ret,"SendHead Semaphore Timeout\n");
    }while (ESPRes == ESP_RES_RSNT);
    ESPState = ESP_STATE_IDLE;
    return ret;
}

void ESPSend(uint8_t *data, size_t len)
{
    if (NetworkState != NETWORK_READY || UDPState != UDP_READY)
        return ;
    ESPClear();
    if (!ESPSendHead(len))
        return;
    ESPState = ESP_STATE_SEND;
    do{
        ESPRes = ESP_RES_NONE;
        HAL_UART_Transmit(&huart1,data,len,2000);
        ITMassert(xSemaphoreTake(ESPRetHandle,pdMS_TO_TICKS(2000)),"Send Semaphore Timeout\n");
    }while (ESPRes == ESP_RES_RSNT);
    ESPState = ESP_STATE_IDLE;
    // ESPClose();
}