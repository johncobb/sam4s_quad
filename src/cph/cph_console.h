#ifndef CPH_CONSOLE_H_
#define CPH_CONSOLE_H_

#include <asf.h>

#define CS_NOINPUT      0
#define CS_INVALID      1
#define CS_PWMMIN       2
#define CS_PWMMID       3
#define CS_PWMMAX       4
#define CS_PWMSTEPINC   5
#define CS_PWMSTEPDEC   6


uint8_t cph_console_tick(void);

#endif