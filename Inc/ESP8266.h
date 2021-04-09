#include <stdio.h>
#include <string.h>
#include "usart.h"
#include "FreeRTOS.h"
#include "task.h"
#include "main.h"
#include "cmsis_os.h"

#define ESP8266_UDP_HEAD "AT+CIPSENDEX="
#define ESP8266_UDP_TAIL "+++\r\n"
#define ESP8266_UDP_OPEN "AT+CIPSTART=\"UDP\",\"192.168.137.1\",5555,1926\r\n"
#define ESP8266_WIFI_CON "AT+CWJAP_DEF=\"DESKTOP-KDLUQ7R 5100\",\"12435687\"\r\n"
#define ESP8266_WIFI_DIS "ATE0\r\n"
#define ESP8266_UDP_MAXL REP(5,3,0,"a")

#define ESP8266_STATE_IDLE 0
#define ESP8266_STATE_INIT 1
#define ESP8266_STATE_CNNT 2
#define ESP8266_STATE_OPEN 3
#define ESP8266_STATE_HEAD 4
#define ESP8266_STATE_SEND 5

void ESP8266Init();
void ESP8266Connect();
void ESP8266SendHead(size_t len);
void ESP8266Open();
void ESP8266Send(uint8_t *data, size_t len);