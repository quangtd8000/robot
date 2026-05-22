#include <avr/interrupt.h>
#include <avr/io.h>
#include <stdbool.h>
#include <util/delay.h>
#include "adc.h"
#include "config.h"
#include "motor.h"
#include "robot.h"
#include "sensors.h"
#include "timer.h"
#include "uart.h"

static void start_button_init(void)
{
    START_BUTTON_DDR &= (uint8_t)~(1U << START_BUTTON_PIN);
    START_BUTTON_PORT |= (1U << START_BUTTON_PIN);
}

static bool start_button_pressed(void)
{
    if (START_BUTTON_INPUT & (1U << START_BUTTON_PIN)) {
        return false;
    }

    _delay_ms(30);

    if (START_BUTTON_INPUT & (1U << START_BUTTON_PIN)) {
        return false;
    }

    while (!(START_BUTTON_INPUT & (1U << START_BUTTON_PIN))) {
    }
    _delay_ms(100);

    return true;
}

int main(void)
{
    uint32_t lastControlMs;

    cli();

    timer2_init_1ms();
    uart_init();
    adc_init();
    motor_init();
    debug_led_init();
    start_button_init();

    sei();

    motor_stop();
    UART_PUTS("\r\nBOOT ATMEGA328P\r\n");
    UART_PUTS("WAIT PD2 START\r\n");

    while (!start_button_pressed()) {
        debug_led_update();
    }

    UART_PUTS("START: CAL 1S\r\n");
    sensors_init();
    robot_init();

    lastControlMs = millis();

    while (1) {
        debug_led_update();

        if (elapsed(lastControlMs) >= CONTROL_DT_MS) {
            lastControlMs += CONTROL_DT_MS;
            sensors_update();
            robot_update();
        }
    }
}
