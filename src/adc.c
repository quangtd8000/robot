#include <avr/io.h>
#include "adc.h"
#include "config.h"

void adc_init(void)
{
    ADMUX = (1U << REFS0);
    ADCSRA = (1U << ADEN) | (1U << ADPS1) | (1U << ADPS0);
}

uint16_t adc_read(uint8_t ch)
{
    ADMUX = (1U << REFS0) | (ch & 0x0FU);

    ADCSRA |= (1U << ADSC);
    while (ADCSRA & (1U << ADSC)) {
    }

    ADCSRA |= (1U << ADSC);

    while (ADCSRA & (1U << ADSC)) {
    }

    return ADC;
}

uint16_t adc_read_avg(uint8_t ch)
{
    uint32_t sum = 0;

    for (uint8_t i = 0; i < ADC_AVG_SAMPLES; i++) {
        sum += adc_read(ch);
    }

    return (uint16_t)(sum / ADC_AVG_SAMPLES);
}
