#ifndef UART_H
#define UART_H

#include <stdint.h>
#include <avr/pgmspace.h>

void uart_init(void);
void uart_putchar(char data);
void uart_puts(const char *s);
void uart_puts_p(const char *s);
void uart_put_u8(uint8_t value);
void uart_put_u16(uint16_t value);
void uart_put_i16(int16_t value);
void uart_put_u32(uint32_t value);

#define UART_PUTS(s) uart_puts_p(PSTR(s))

#endif
