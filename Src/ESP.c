#include "ESP.h"
#include "gpio.h"
#include "spi.h"
#include "dma.h"
#include "usart.h"
#include "FreeRTOS.h"
#include "task.h"

TaskHandle_t ESPTaskHandle = NULL;
uint8_t lraw[3];
uint8_t ESPState = ESP_NRST;
uint8_t clear[] = {ESP_IGN, ESP_END, ESP_END, ESP_END};
int8_t fails = 0;

uint16_t ESPCalcParity(uint16_t len)
{
    uint8_t odd,even,all;
    odd = even = all = 0;
    for (uint8_t i=0;i<9;i++)
    {
        odd ^= i & 1 & ( (len>>i) & 1);
        even ^= ((i & 1) ^ 1) & ( (len>>i) & 1);
        all ^= ( (len>>i) & 1);
    }
    return (len | (odd << 9) | (even << 10) | (all << 11));
}

uint8_t ESPSend(uint8_t *raw,size_t len)
{
    if (ESPState == ESP_NRST)
    {
        ESPInit();
        if (ESPState == ESP_NRST)
            return ESP_FAIL;
    }
    ESPTaskHandle = xTaskGetCurrentTaskHandle();
    len = ESPCalcParity(len);
    *((uint16_t *)lraw) = len;
    ESPState = ESP_SLEN;
    HAL_UART_Transmit(&espuart,lraw,3,500);
    // itm_printf("UART?\n");
    if (ulTaskNotifyTake(pdTRUE,pdMS_TO_TICKS(800)) == pdFALSE)
        return ESPClear(),ESP_FAIL;
    fails = MAX(0,fails-2);
    // itm_printf("UART!\n");
    ESPState = ESP_SSPI;
    HAL_SPI_Transmit_DMA(&espspi,raw,len);
    HAL_GPIO_WritePin(ESP_notify_GPIO_Port,ESP_notify_Pin,GPIO_PIN_SET);
    // itm_printf("SPI?\n");
    if (ulTaskNotifyTake(pdTRUE,pdMS_TO_TICKS(800)) == pdFALSE)
        return ESPInit(),ESP_FAIL;
    // itm_printf("SPI!\n");
    ESPState = ESP_IDLE;
    HAL_GPIO_WritePin(ESP_notify_GPIO_Port,ESP_notify_Pin,GPIO_PIN_RESET);
    ESPTaskHandle = NULL;
    return ESP_OK;
}

void ESPClear()
{
    if (++fails > ESP_MAX_FAIL)
    {
        ESPInit();
        fails = 0;
        return ;
    }
    itm_printf("Throw!\n");
    HAL_UART_Transmit(&espuart,clear,sizeof(clear),500);
}


void ESPInit()
{
    ESPTaskHandle = xTaskGetCurrentTaskHandle();
    lraw[2] = ESP_END;
    HAL_GPIO_WritePin(ESP_reset_GPIO_Port,ESP_reset_Pin,GPIO_PIN_RESET);
    HAL_Delay(10);
    ESPState = ESP_BOOT;
    HAL_GPIO_WritePin(ESP_reset_GPIO_Port,ESP_reset_Pin,GPIO_PIN_SET);
    itm_printf("Reset OK?\n");
    // while (HAL_GPIO_ReadPin(ESP_handshake_GPIO_Port,ESP_handshake_Pin)==GPIO_PIN_RESET);
    // if (ulTaskNotifyTake(pdTRUE,pdMS_TO_TICKS(5000)) == pdTRUE)
    if (ulTaskNotifyTake(pdTRUE,portMAX_DELAY) == pdTRUE)
    {
        ESPState = ESP_IDLE;
        itm_printf("Reset OK!\n");
    }
    else
        ESPState = ESP_NRST;
    ESPTaskHandle = NULL;
    return ;
}

void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
    BaseType_t Woken = pdFALSE;
    if (GPIO_Pin == ESP_handshake_Pin)
    {
        uint8_t state = HAL_GPIO_ReadPin(ESP_handshake_GPIO_Port,ESP_handshake_Pin);

        if (ESPState == ESP_NRST)
            return ;
        if (ESPTaskHandle == NULL)
            itm_printf("What the fuck?ESP_handshake pull up without any transmission\n%d\n",ESPState);
        else if (state == GPIO_PIN_RESET)
        {
            if (ESPState == ESP_SLEN)
                vTaskNotifyGiveFromISR(ESPTaskHandle,&Woken);
            else
                itm_printf("LOW:%d\n",ESPState);
        }
        else if (state == GPIO_PIN_SET)
        {
            if (ESPState == ESP_SSPI || ESPState == ESP_BOOT)
                vTaskNotifyGiveFromISR(ESPTaskHandle,&Woken);
            else
                itm_printf("HIGH:%d\n",ESPState);
        }
        else
            itm_printf("How?\n");
    }
    portYIELD_FROM_ISR(Woken);
    // HAL_GPIO_TogglePin(LED0_GPIO_Port,LED0_Pin);
}