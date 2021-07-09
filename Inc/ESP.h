#include "main.h"

#define ESP_IGN 0xFE
#define ESP_END 0xFF

#define ESP_IDLE 0
#define ESP_NRST 1
#define ESP_BOOT 2
#define ESP_SLEN 3
#define ESP_SSPI 4

#define ESP_MAX_FAIL 50
#define ESP_FAIL 0
#define ESP_OK 1

#define espuart huart1
#define espspi hspi1

void ESPInit();
void ESPClear();
uint8_t ESPSend(uint8_t *raw,size_t len);
uint16_t ESPCalcParity(uint16_t len);
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin);

