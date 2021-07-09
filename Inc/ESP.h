#include "main.h"

#define ESP_BOUND  "\xED\x4D\xB9\x6F"
#define UART_BEGIN "\xDE\xDA\xBE\xFE"
#define UART_END   "\xED\xF4"

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
uint8_t ESPSend(uint8_t *raw,size_t len);

