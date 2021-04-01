#include <_ansi.h>
#include <_syslist.h>
#include <errno.h>
#include <sys/time.h>
#include <sys/times.h>
#include <retarget.h>
#include <stdint.h>
#include <stdio.h>
#include <stdarg.h>

#if !defined(OS_USE_SEMIHOSTING)

#define STDIN_FILENO  0
#define STDOUT_FILENO 1
#define STDERR_FILENO 2

volatile int32_t ITM_RxBuffer=ITM_RXBUFFER_EMPTY;

// uint32_t vprint(const char *fmt, va_list argp)
// {
//     char s[0x80];
//     uint32_t len=vsprintf(s,fmt,argp);
//     for (uint32_t i=0;i<len;i++)
//         ITM_SendChar(s[i]);
//     return len;
// }

// uint32_t itm_printf(const char *fmt, ...)
// {
//     va_list argp;
//     va_start(argp, fmt);
//     uint32_t t=vprint(fmt, argp);
//     va_end(argp);
//     return t;
// }

void RetargetInit()
{
    /* Disable I/O buffering for STDOUT stream, so that
     * chars are sent out as soon as they are printed. */
    setvbuf(stdout, NULL, _IONBF, 0);
}

int _isatty(int fd)
{
    if (fd >= STDIN_FILENO && fd <= STDERR_FILENO)
        return 1;

    errno = EBADF;
    return 0;
}

int _write(int fd, char *ptr, int len)
{
    ITM_SendChar(*ptr);
    return 1;
}

int _close(int fd)
{
    if (fd >= STDIN_FILENO && fd <= STDERR_FILENO)
        return 0;

    errno = EBADF;
    return -1;
}

int _lseek(int fd, int ptr, int dir)
{
    (void) fd;
    (void) ptr;
    (void) dir;

    errno = EBADF;
    return -1;
}

int _read(int fd, char *ptr, int len)
{
    int32_t c = ITM_ReceiveChar();
    if (c==-1)
        return -1;
    *ptr = c;
    return 1;
}

int _fstat(int fd, struct stat *st)
{
    if (fd >= STDIN_FILENO && fd <= STDERR_FILENO)
    {
        st->st_mode = S_IFCHR;
        return 0;
    }

    errno = EBADF;
    return 0;
}

#endif