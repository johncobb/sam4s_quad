#ifndef CPH_MILLIS_H_
#define CPH_MILLIS_H_

#include <asf.h>

typedef uint32_t clock_time_t;

extern volatile uint32_t g_cph_millis;

#define cph_get_millis(void)	g_cph_millis

void cph_millis_init(void);
void cph_millis_delay(uint32_t millis);

#endif