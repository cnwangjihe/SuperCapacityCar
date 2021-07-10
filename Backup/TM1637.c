/*______________In the name of Allah, Most Gracious, Most Merciful______________
	
	This library is written to allow the STM32 micro-controller to handle TM1637 LED
	driver using a manual communication protocol (through GPIO pins)
	
	Created by 				: cnwangjihe
	Start Date 				: 2021/03/12
	file name 				: TM1637.c
	Version					: 1.1
______________________________________________________________________________*/

#ifndef tm1637
#include "TM1637.h"

void TM1637_Switch(TM1637 *t, uint8_t Direction)								//Since SDI line is doing both transmission and reception
{																		//the corresponding GPIO pin must be reinitialized on the run
	GPIO_InitTypeDef GPIO_InitStruct = {0};								//To read ACK from TM1637 and to write data to it
	GPIO_InitStruct.Pin = t->SDO_Pin;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
	if (Direction == DISPLAY2STM)
		GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
	else if (Direction == STM2DISPLAY)
		GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
	else
		return ;
	HAL_GPIO_Init(t->SDO_GPIO_Port, &GPIO_InitStruct);
}

void TM1637_Init(TM1637 *t)								//Since SDI line is doing both transmission and reception
{																		//the corresponding GPIO pin must be reinitialized on the run
	GPIO_InitTypeDef GPIO_InitStruct = {0};								//To read ACK from TM1637 and to write data to it
	GPIO_InitStruct.Pin = t->SCLK_Pin;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
	GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
	HAL_GPIO_Init(t->SCLK_GPIO_Port, &GPIO_InitStruct);

	GPIO_InitStruct.Pin = t->SDO_Pin;
	HAL_GPIO_Init(t->SDO_GPIO_Port, &GPIO_InitStruct);
}


void TM1637_CLKhigh(TM1637 *t) {HAL_GPIO_WritePin(t->SCLK_GPIO_Port, t-> SCLK_Pin, GPIO_PIN_SET);}
void TM1637_CLKlow(TM1637 *t) {HAL_GPIO_WritePin(t->SCLK_GPIO_Port, t->SCLK_Pin, GPIO_PIN_RESET);}
void TM1637_SDOhigh(TM1637 *t) {HAL_GPIO_WritePin(t->SDO_GPIO_Port, t->SDO_Pin, GPIO_PIN_SET);}
void TM1637_SDOlow(TM1637 *t) {HAL_GPIO_WritePin(t->SDO_GPIO_Port, t->SDO_Pin, GPIO_PIN_RESET);}


void TM1637_Start(TM1637 *t)
{
	TM1637_CLKhigh(t);
	TM1637_SDOhigh(t);

	TM1637_SDOlow(t);
}


void TM1637_End(TM1637 *t)
{
	TM1637_CLKlow(t);
	TM1637_SDOlow(t);

	TM1637_CLKhigh(t);
	TM1637_SDOhigh(t);
}


void TM1637_DataOut(TM1637 *t,uint8_t *TM1637_TxBuffer)							//Low level data transfer function
{
	for(int8_t j = 0; j < PACKET_SIZE; j++)								//Send least significant bit first
	{
		TM1637_CLKlow(t);
		if (TM1637_TxBuffer[j] == GPIO_PIN_SET)							//Check logic level
			TM1637_SDOhigh(t);
		else
			TM1637_SDOlow(t);
		TM1637_CLKhigh(t);
	} 
}


void TM1637_ACKcheck(TM1637 *t)
{
	//Wait for acknowledgment bit
	TM1637_Switch(t, DISPLAY2STM);										//initialize pin as input
	TM1637_CLKlow(t);													//lower CLK line
	while (HAL_GPIO_ReadPin(t->SDO_GPIO_Port, t->SDO_Pin));					//Wait until ACK bit is received
	TM1637_Switch(t, STM2DISPLAY);										//initialize pin as output for data transfer
	TM1637_CLKhigh(t);
}

void TM1637_TxCommand(TM1637 *t,uint8_t *Command)
{																		//Handles high level (bit by bit) transmission operation
	uint8_t ByteData[8] = {0};

	for(uint8_t i = 0; i < PACKET_SIZE; i++)
		ByteData[i] = (Command[0] & (0x01 << i)) && 1;					//Convert from byte to bit per array element

	TM1637_Start(t);												//Send start packet bit
	TM1637_DataOut(t, ByteData);											//Send one byte
	TM1637_ACKcheck(t);													//wait for acknowledgment.
	if ((Command[0] & 0xC0) != (0xC0))										//Check if the received packet is not an address.
		TM1637_End(t);
}


void TM1637_TxData(TM1637 *t,uint8_t *Data, uint8_t PacketSize)			//Handles high level (bit by bit) transmission operation
{
	uint8_t ByteData[8] = {0};
	TM1637_Start(t);
	for(uint8_t i = 0; i < PacketSize; i++)
	{
		for(uint8_t j = 0; j < 8; j++)
			ByteData[j] = (Data[i] & (0x01 << j)) && 1;
		TM1637_DataOut(t, ByteData);
		TM1637_ACKcheck(t);												//Transmit byte by byte
	}
	TM1637_End(t);													//Send end packet at the end of data transmission.
}


void TM1637_DisplayClear(TM1637 *t)
{
	uint8_t EmptyBuffer[4] = {0};
	uint8_t CommandCarrier[1] = {0};
	CommandCarrier[0] = DATA_SET;									//Send set data command
	TM1637_TxCommand(t, CommandCarrier);
	CommandCarrier[0] = C0H;										//Set address
	TM1637_TxCommand(t, CommandCarrier);
	TM1637_TxData(t, EmptyBuffer, 4);
	CommandCarrier[0] = DISPLAY_OFF;
	TM1637_TxCommand(t, CommandCarrier);
}


void TM1637_SetBrighness(TM1637 *t, uint8_t BrighnessLevel)
{
	uint8_t BrighnessBuffer[8] = {0};
	if (BrighnessLevel > 7)												//there are 7 levels of brightness
		BrighnessLevel = 7;

	BrighnessLevel = BrighnessLevel | DISPLAY_ON;					//Set Brightness level with display on command

	for(uint8_t i = 0; i < 8; i++)
		BrighnessBuffer[i] = (BrighnessLevel & (0x01 << i)) && 1;
	TM1637_Start(t);
	TM1637_DataOut(t, BrighnessBuffer);
	TM1637_ACKcheck(t);
	TM1637_End(t);
}


void TM1637_DisplaySend(TM1637 *t, uint8_t Brightness, uint8_t *DisplayBuffer)
{
	//This function handles the low level protocol used to set data address of TM1637 and turn the display on
	//#param Brightness is used to set the brightness level of the display. This function accepts Brightness value between 0 and 7
	//#param *DisplayBuffer is the buffer used to map data from the RAM to the display each element corresponds to one segment in the display
	uint8_t CommandCarrier[1] = {0};
	if (Brightness > 7)
        Brightness = 7;
	CommandCarrier[0] = DATA_SET;									//Send set data command
	TM1637_TxCommand(t, CommandCarrier);
	CommandCarrier[0] = C0H;										//Set address
	TM1637_TxCommand(t, CommandCarrier);

	TM1637_TxData(t, DisplayBuffer, 4);								//Map the data stored in RAM to the display
	TM1637_SetBrighness(t, Brightness);								//Turn on display and set brightness
}

void TM1637_Display(TM1637 *t, uint8_t Brightness, char *DisplayBuffer, uint8_t Point)
{
	int len = sizeof(DisplayBuffer);
	uint8_t *Buffer = (uint8_t*)malloc(len);
	for (uint8_t i=0;i!=len;i++)
		Buffer[i]=TM1637_char2segments(DisplayBuffer[i]) | Point;
	TM1637_DisplaySend(t,Brightness,Buffer);
	free(Buffer);
}

uint8_t TM1637_char2segments(char c)
{
    switch (c)
	{
		case '0' : return 0x3f;
		case '1' : return 0x06;
		case '2' : return 0x5b;
		case '3' : return 0x4f;
		case '4' : return 0x66;
		case '5' : return 0x6d;
		case '6' : return 0x7d;
		case '7' : return 0x07;
		case '8' : return 0x7f;
		case '9' : return 0x6f;
        case '_' : return 0x08;
        case '^' : return 0x01; // ¯
        case '-' : return 0x40;
        case '*' : return 0x63; // °
        case ' ' : return 0x00; // space
        case 'A' : return 0x77; // upper case A
        case 'a' : return 0x5f; // lower case a
        case 'B' :              // lower case b
        case 'b' : return 0x7c; // lower case b
        case 'C' : return 0x39; // upper case C
        case 'c' : return 0x58; // lower case c
        case 'D' :              // lower case d
        case 'd' : return 0x5e; // lower case d
        case 'E' :              // upper case E
        case 'e' : return 0x79; // upper case E
        case 'F' :              // upper case F
        case 'f' : return 0x71; // upper case F
        case 'G' :              // upper case G
        case 'g' : return 0x35; // upper case G
        case 'H' : return 0x76; // upper case H
        case 'h' : return 0x74; // lower case h
        case 'I' : return 0x06; // 1
        case 'i' : return 0x04; // lower case i
        case 'J' : return 0x1e; // upper case J
        case 'j' : return 0x16; // lower case j
        case 'K' :              // upper case K
        case 'k' : return 0x75; // upper case K
        case 'L' :              // upper case L
        case 'l' : return 0x38; // upper case L
        case 'M' :              // twice tall n
        case 'm' : return 0x37; // twice tall ∩
        case 'N' :              // lower case n
        case 'n' : return 0x54; // lower case n
        case 'O' :              // lower case o
        case 'o' : return 0x5c; // lower case o
        case 'P' :              // upper case P
        case 'p' : return 0x73; // upper case P
        case 'Q' : return 0x7b; // upper case Q
        case 'q' : return 0x67; // lower case q
        case 'R' :              // lower case r
        case 'r' : return 0x50; // lower case r
        case 'S' :              // 5
        case 's' : return 0x6d; // 5
        case 'T' :              // lower case t
        case 't' : return 0x78; // lower case t
        case 'U' :              // lower case u
        case 'u' : return 0x1c; // lower case u
        case 'V' :              // twice tall u
        case 'v' : return 0x3e; // twice tall u
        case 'W' : return 0x7e; // upside down A
        case 'w' : return 0x2a; // separated w
        case 'X' :              // upper case H
        case 'x' : return 0x76; // upper case H
        case 'Y' :              // lower case y
        case 'y' : return 0x6e; // lower case y
        case 'Z' :              // separated Z
        case 'z' : return 0x1b; // separated Z
    }
    return 0;
}
#endif
