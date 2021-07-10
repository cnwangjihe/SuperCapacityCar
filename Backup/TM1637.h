/*______________In the name of Allah, Most Gracious, Most Merciful______________
	
	This library is written to allow the STM32 micro-controller to handle TM1637 LED
	driver using a manual communication protocol (through GPIO pins)
	
	Created by 				: cnwangjihe
	Start Date 				: 2021/03/12
	file name 				: TM1637.h
	Version					: 1.1
______________________________________________________________________________*/
#ifndef tm1637
#define tm1637

#include "stm32f4xx_hal.h"
#include <malloc.h>

//==============================================================================
// Definitions 
//==============================================================================

typedef struct
{
	GPIO_TypeDef *SCLK_GPIO_Port;
	uint16_t SCLK_Pin;
	GPIO_TypeDef *SDO_GPIO_Port;
	uint16_t SDO_Pin;
}TM1637;

#define A_SEG				0x01
#define B_SEG				0x02
#define C_SEG				0x04
#define D_SEG				0x08
#define E_SEG				0x10
#define F_SEG				0x20
#define G_SEG				0x40
#define DP_SEG				0x80

//Settings
#define STM2DISPLAY			0x00
#define DISPLAY2STM			0x01


//Display Address
#define C0H					0xC0
#define C1H					0xC1
#define C2H					0xC2
#define C3H					0xC3
#define C4H					0xC4
#define C5H					0xC5
//Commands
#define DATA_SET			0x40
#define DATA_FIXED			0x44
#define DISPLAY_ON			0x88
#define DISPLAY_OFF			0x80
#define PACKET_SIZE			0x08

#define TM1637_POINT_ON		0x80
#define TM1637_POINT_OFF	0x00

//==============================================================================
// Functions Declaration 
//==============================================================================
void TM1637_Switch(TM1637 *t, uint8_t Direction);
void TM1637_Init(TM1637 *t);
void TM1637_CLKhigh(TM1637 *t);
void TM1637_CLKlow(TM1637 *t);
void TM1637_SDOhigh(TM1637 *t);
void TM1637_SDOlow(TM1637 *t);
void TM1637_Start(TM1637 *t);
void TM1637_End(TM1637 *t);
void TM1637_DataOut(TM1637 *t,uint8_t *TM1637_TxBuffer);
void TM1637_ACKcheck(TM1637 *t);
void TM1637_TxCommand(TM1637 *t,uint8_t *Command);
void TM1637_TxData(TM1637 *t,uint8_t *Data, uint8_t PacketSize);
void TM1637_DisplayClear(TM1637 *t);
void TM1637_SetBrighness(TM1637 *t, uint8_t BrighnessLevel);
void TM1637_DisplaySend(TM1637 *t, uint8_t Brightness, uint8_t *DisplayBuffer);
void TM1637_Display(TM1637 *t, uint8_t Brightness, char *DisplayBuffer, uint8_t Point);
uint8_t TM1637_char2segments(char c);
#endif
