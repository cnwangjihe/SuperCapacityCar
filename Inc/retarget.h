#ifndef _RETARGET_H__
#define _RETARGET_H__
#include "stm32f4xx_hal.h"
#include <sys/stat.h>
#include <stdio.h>

void RetargetInit();

// uint32_t vprint(const char *fmt, va_list argp);

// uint32_t itm_printf(const char *fmt, ...);

int _isatty(int fd);

int _write(int fd, char *ptr, int len);

int _close(int fd);

int _lseek(int fd, int ptr, int dir);

int _read(int fd, char *ptr, int len);

int _fstat(int fd, struct stat *st);

#endif