#include "ESP.h"
#include "gpio.h"
#include "spi.h"
#include "dma.h"
#include "usart.h"
#include "FreeRTOS.h"
#include "task.h"

uint8_t ESPSend(uint8_t *raw,size_t len)
{
    HAL_UART_Transmit(&espuart,raw,len,300);
    HAL_UART_Transmit(&espuart,(uint8_t *)ESP_BOUND,sizeof(ESP_BOUND-1),300);
    return ESP_OK;
}

void ESPInit()
{
    HAL_UART_Transmit(&espuart,(uint8_t *)ESP_BOUND,sizeof(ESP_BOUND-1),300);
    return ;
}