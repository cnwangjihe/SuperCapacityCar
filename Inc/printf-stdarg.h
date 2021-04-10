#include "stm32f4xx_hal.h"


#define putchar(c) ITM_SendChar(c)

int printf(const char *format, ...);
int sprintf(char *out, const char *format, ...);
int snprintf( char *buf, unsigned int count, const char *format, ... );
int	write( int i, char* c, int n);