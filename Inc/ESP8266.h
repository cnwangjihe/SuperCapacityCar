// #include "printf-stdarg.h"
#include "main.h"

#define ESP_UDP_HEAD "AT+CIPSENDEX="
#define ESP_UDP_TAIL "+++\r\n"
#define ESP_UDP_CLOS "AT+CIPCLOSE\r\n"
#define ESP_UDP_OPEN "AT+CIPSTART=\"UDP\",\"192.168.137.1\",5555,1926\r\n"
#define ESP_WIFI_CON "AT+CWJAP=\"DESKTOP-KDLUQ7R 5100\",\"12435687\"\r\n"
#define ESP_WIFI_DIS "ATE0\r\n"
#define ESP_UDP_MAXL REP(5,3,0,"a")

#define ESP_STATE_IDLE 0
#define ESP_STATE_INIT 1
#define ESP_STATE_CNNT 2
#define ESP_STATE_OPEN 3
#define ESP_STATE_HEAD 4
#define ESP_STATE_SEND 5
#define ESP_STATE_CLOS 6

#define ESP_RES_NONE 0
#define ESP_RES_RSNT 1
#define ESP_RES_CLOS 2 

#define ESP_SPI_NULL 0
#define ESP_SPI_WRITE 1
#define ESP_SPI_READ 2

void ESPInit();
void ESPConnect();
uint8_t ESPSendHead(size_t len);
void ESPOpen();
void ESPSend(uint8_t *data, size_t len);
// inline void ESP_Interrupt();