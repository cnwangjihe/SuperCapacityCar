#include "ESP8266.h"

// extern osSemaphoreId UDPHeadFinHandle,UDPSendFinHandle,WifiConFinHandle,UDPOpenFinHandle,WifiDisFinHandle;
extern osSemaphoreId ESP8266RetHandle;
uint8_t UDPState, NetworkState;
uint8_t ESP8266State, ESP8266Resend;

void ESP8266Connect()
{
    ESP8266State = ESP8266_STATE_CNNT;
    do{
        ESP8266Resend = 0;
        HAL_UART_Transmit(&huart1,(uint8_t *)"\r\n\r\n",4,200);
        HAL_UART_Transmit(&huart1,(uint8_t *)ESP8266_WIFI_CON,48,200);
        // ITMassert(xSemaphoreTake(WifiConFinHandle,pdMS_TO_TICKS(50000)),"Connet Semaphore Timeout");
        ITMassert(xSemaphoreTake(ESP8266RetHandle,pdMS_TO_TICKS(50000)),"Connet Semaphore Timeout");
    }while (ESP8266Resend);
    ESP8266State = ESP8266_STATE_IDLE;
}

void ESP8266Init()
{
    ESP8266State = ESP8266_STATE_INIT;
    do{
        ESP8266Resend = 0;
        HAL_UART_Transmit(&huart1,(uint8_t *)"\r\n\r\n",4,200);
        HAL_UART_Transmit(&huart1,(uint8_t *)ESP8266_WIFI_DIS,6,200);
        // ITMassert(xSemaphoreTake(WifiDisFinHandle,pdMS_TO_TICKS(500)),"Init Semaphore Timeout");
        ITMassert(xSemaphoreTake(ESP8266RetHandle,pdMS_TO_TICKS(500)),"Init Semaphore Timeout");
    }while (ESP8266Resend);
    ESP8266State = ESP8266_STATE_IDLE;
}

void ESP8266Open()
{
    ESP8266State = ESP8266_STATE_OPEN;
    do{
        ESP8266Resend = 0;
        HAL_UART_Transmit(&huart1,(uint8_t *)"\r\n\r\n",4,200);
        HAL_UART_Transmit(&huart1,(uint8_t *)ESP8266_UDP_OPEN,45,200);
        // ITMassert(xSemaphoreTake(UDPOpenFinHandle,pdMS_TO_TICKS(500)),"Open Semaphore Timeout");
        ITMassert(xSemaphoreTake(ESP8266RetHandle,pdMS_TO_TICKS(500)),"Open Semaphore Timeout");
    }while (ESP8266Resend);
    ESP8266State = ESP8266_STATE_IDLE;
}

void ESP8266SendHead(size_t len)
{
    char tmp[8];
    sprintf(tmp,"%d\r\n",len);
    ESP8266State = ESP8266_STATE_HEAD;
    do{
        ESP8266Resend = 0;
        HAL_UART_Transmit(&huart1,(uint8_t *)"\r\n\r\n",4,200);
        HAL_UART_Transmit(&huart1,(uint8_t *)ESP8266_UDP_HEAD,13,200);
        HAL_UART_Transmit(&huart1,(uint8_t *)tmp,strlen(tmp),200);
        // ITMassert(xSemaphoreTake(UDPHeadFinHandle,pdMS_TO_TICKS(500)),"SendHead Semaphore Timeout");
        ITMassert(xSemaphoreTake(ESP8266RetHandle,pdMS_TO_TICKS(5000)),"SendHead Semaphore Timeout");
    }while (ESP8266Resend);
    ESP8266State = ESP8266_STATE_IDLE;
}

void ESP8266Send(uint8_t *data, size_t len)
{
    if (NetworkState != NETWORK_READY)
        ESP8266Connect();
    if (NetworkState != NETWORK_READY)
        return ;
    if (UDPState != UDP_READY)
        ESP8266Open();
    if (UDPState != UDP_READY)
        return ;
    ESP8266SendHead(len);
    ESP8266State = ESP8266_STATE_SEND;
    do{
        ESP8266Resend = 0;
        HAL_UART_Transmit(&huart1,data,len,800);
        // ITMassert(xSemaphoreTake(UDPSendFinHandle,pdMS_TO_TICKS(100)),"Send Semaphore Timeout");
        ITMassert(xSemaphoreTake(ESP8266RetHandle,pdMS_TO_TICKS(100)),"Send Semaphore Timeout");
    }while (ESP8266Resend);
    ESP8266State = ESP8266_STATE_IDLE;
    // ESP8266Close();
}