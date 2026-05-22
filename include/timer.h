#ifndef TIMER_H
#define TIMER_H

#include <stdint.h>

extern volatile uint32_t g_ms;

void timer2_init_1ms(void);
uint32_t millis(void);
uint32_t elapsed(uint32_t start);

#endif
