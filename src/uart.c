#include <avr/io.h>
#include <avr/pgmspace.h>
#include "config.h"
#include "uart.h"

void uart_init(void)
{
    uint16_t ubrr = (uint16_t)(F_CPU / 8UL / UART_BAUD - 1UL);

    UBRR0H = (uint8_t)(ubrr >> 8);
    UBRR0L = (uint8_t)ubrr;
    UCSR0A = (1U << U2X0);
    UCSR0B = (1U << TXEN0);
    UCSR0C = (1U << UCSZ01) | (1U << UCSZ00);
}

void uart_putchar(char data)
{
    while (!(UCSR0A & (1U << UDRE0))) {
    }
    UDR0 = data;
}

void uart_puts(const char *s)
{
    while (*s) {
        uart_putchar(*s++);
    }
}

void uart_puts_p(const char *s)
{
    char c;

    while ((c = (char)pgm_read_byte(s++)) != '\0') {
        uart_putchar(c);
    }
}

void uart_put_u32(uint32_t value)
{
    char buf[10];
    uint8_t i = 0;

    if (value == 0UL) {
        uart_putchar('0');
        return;
    }

    while (value > 0UL) {
        buf[i++] = (char)('0' + (value % 10UL));
        value /= 10UL;
    }

    while (i > 0U) {
        uart_putchar(buf[--i]);
    }
}

void uart_put_u16(uint16_t value)
{
    uart_put_u32(value);
}

void uart_put_u8(uint8_t value)
{
    uart_put_u32(value);
}

void uart_put_i16(int16_t value)
{
    if (value < 0) {
        uart_putchar('-');
        value = (int16_t)-value;
    }

    uart_put_u16((uint16_t)value);
}
