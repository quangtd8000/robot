#include <avr/interrupt.h>
#include <avr/io.h>
#include "timer.h"

volatile uint32_t g_ms;

ISR(TIMER2_COMPA_vect)
{
    g_ms++;
}

void timer2_init_1ms(void)
{
    TCCR2A = 0;
    TCCR2B = 0;
    TCNT2 = 0;
    OCR2A = 124;
    TCCR2A |= (1U << WGM21);
    TCCR2B |= (1U << CS21);
    TIMSK2 |= (1U << OCIE2A);
}

uint32_t millis(void)
{
    uint32_t m;
    uint8_t sreg = SREG;

    cli();
    m = g_ms;
    SREG = sreg;

    return m;
}

uint32_t elapsed(uint32_t start)
{
    return millis() - start;
}
