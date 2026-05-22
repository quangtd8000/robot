#ifndef ADC_H
#define ADC_H

#include <stdint.h>

void adc_init(void);
uint16_t adc_read(uint8_t ch);
uint16_t adc_read_avg(uint8_t ch);

#endif
