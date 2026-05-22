#include <stdbool.h>
#include <stdint.h>
#include <avr/io.h>
#include "config.h"
#include "motor.h"

static int16_t clamp_speed(int16_t speed)
{
    if (speed > PWM_MAX) {
        return PWM_MAX;
    }
    if (speed < -PWM_MAX) {
        return -PWM_MAX;
    }
    return speed;
}

static uint8_t speed_abs(int16_t speed)
{
    if (speed < 0) {
        return (uint8_t)(-speed);
    }
    return (uint8_t)speed;
}

static uint8_t apply_min_pwm(uint8_t pwm, uint8_t minPwm)
{
    if (pwm == 0U) {
        return 0;
    }
    if (pwm < minPwm) {
        return minPwm;
    }
    return pwm;
}

static void set_pin(volatile uint8_t *port, uint8_t pin, bool high)
{
    if (high) {
        *port |= (uint8_t)(1U << pin);
    } else {
        *port &= (uint8_t)~(1U << pin);
    }
}

static void set_left_direction(bool forward)
{
#if MOTOR_IN2_TIED_GND
    bool dirHigh = forward;
#if !L_DIR_HIGH_FORWARD
    dirHigh = !dirHigh;
#endif
    set_pin(&L_IN1_PORT, L_IN1, dirHigh);
#else
    set_pin(&L_IN1_PORT, L_IN1, forward);
    set_pin(&L_IN2_PORT, L_IN2, !forward);
#endif
}

static void set_right_direction(bool forward)
{
#if MOTOR_IN2_TIED_GND
    bool dirHigh = forward;
#if !R_DIR_HIGH_FORWARD
    dirHigh = !dirHigh;
#endif
    set_pin(&R_IN1_PORT, R_IN1, dirHigh);
#else
    set_pin(&R_IN1_PORT, R_IN1, forward);
    set_pin(&R_IN2_PORT, R_IN2, !forward);
#endif
}

void motor_init(void)
{
    DDRB |= (uint8_t)((1U << L_PWM_PIN) | (1U << R_PWM_PIN));

    L_IN1_DDR |= (uint8_t)(1U << L_IN1);
    R_IN1_DDR |= (uint8_t)(1U << R_IN1);
#if !MOTOR_IN2_TIED_GND
    L_IN2_DDR |= (uint8_t)(1U << L_IN2);
    R_IN2_DDR |= (uint8_t)(1U << R_IN2);
#endif

    TCCR1A = 0;
    TCCR1B = 0;
    TCNT1 = 0;
    OCR1A = 0;
    OCR1B = 0;

    TCCR1A = (1U << COM1A1) | (1U << COM1B1) | (1U << WGM10);
    TCCR1B = (1U << WGM12) | (1U << CS10);

    motor_stop();
}

void motor_set(int16_t left, int16_t right)
{
    uint8_t leftPwm;
    uint8_t rightPwm;

    left = clamp_speed(left);
    right = clamp_speed(right);

    set_left_direction(left >= 0);
    set_right_direction(right >= 0);

    leftPwm = apply_min_pwm(speed_abs(left), L_MOTOR_MIN_PWM);
    rightPwm = apply_min_pwm(speed_abs(right), R_MOTOR_MIN_PWM);

    OCR1A = leftPwm;
    OCR1B = rightPwm;
}

void motor_stop(void)
{
    OCR1A = 0;
    OCR1B = 0;

    set_pin(&L_IN1_PORT, L_IN1, false);
    set_pin(&R_IN1_PORT, R_IN1, false);
#if !MOTOR_IN2_TIED_GND
    set_pin(&L_IN2_PORT, L_IN2, false);
    set_pin(&R_IN2_PORT, R_IN2, false);
#endif
}
