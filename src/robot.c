#include <stdbool.h>
#include <stdint.h>
#include <avr/io.h>
#include "config.h"
#include "motor.h"
#include "robot.h"
#include "sensors.h"
#include "timer.h"
#include "uart.h"

static RobotState state;
static uint32_t stateEnterMs;
static uint32_t turnStartMs;
static uint32_t debugPulseStartMs;
static bool debugPulseActive;
static int16_t pidI;
static int16_t lastError;
static bool turnbackLogged;

static int16_t clamp_i16(int16_t value, int16_t low, int16_t high)
{
    if (value < low) {
        return low;
    }
    if (value > high) {
        return high;
    }
    return value;
}

static uint16_t near_value(uint16_t value)
{
#if SENSOR_NEAR_IS_HIGH
    return value;
#else
    return (uint16_t)(1023U - value);
#endif
}

static char state_code(RobotState value)
{
    switch (value) {
    case ST_INIT:
        return 'I';
    case ST_FOLLOW_CORRIDOR:
        return 'F';
    case ST_TURN_BACK:
        return 'B';
    case ST_FINISHED:
        return 'X';
    case ST_ERROR:
    default:
        return 'E';
    }
}

static void robot_reset_pid(void)
{
    pidI = 0;
    lastError = 0;
}

static void debug_print_sensor_snapshot(void)
{
    UART_PUTS(" ADC FL=");
    uart_put_u16(g_sensors.fltFL45);
    UART_PUTS(" FR=");
    uart_put_u16(g_sensors.fltFR45);
    UART_PUTS(" L90=");
    uart_put_u16(g_sensors.fltL90);
    UART_PUTS(" R90=");
    uart_put_u16(g_sensors.fltR90);
    UART_PUTS(" TG FL=");
    uart_put_u16(g_sensors.targetFL45);
    UART_PUTS(" FR=");
    uart_put_u16(g_sensors.targetFR45);
    UART_PUTS(" L=");
    uart_put_u16(g_sensors.targetL90);
    UART_PUTS(" R=");
    uart_put_u16(g_sensors.targetR90);
}

static void robot_enter(RobotState next)
{
    RobotState previous = state;

    state = next;
    stateEnterMs = millis();
    sensors_set_debug_enabled(next != ST_TURN_BACK);

    if (previous != next) {
        UART_PUTS("STATE ");
        uart_putchar(state_code(previous));
        UART_PUTS("->");
        uart_putchar(state_code(next));
        UART_PUTS(" t=");
        uart_put_u32(stateEnterMs);
        UART_PUTS("\r\n");
    }

    if (next == ST_FOLLOW_CORRIDOR) {
        robot_reset_pid();
        turnbackLogged = false;
    }

    if (next == ST_TURN_BACK) {
        sensors_reset_node_debounce();
        debugPulseActive = true;
        debugPulseStartMs = stateEnterMs;
    }
}

void robot_init(void)
{
    state = ST_INIT;
    stateEnterMs = millis();
    turnStartMs = stateEnterMs;
    debugPulseStartMs = stateEnterMs;
    debugPulseActive = false;
    turnbackLogged = false;
    robot_reset_pid();
}

void robot_follow_corridor(void)
{
    int16_t flValue = (int16_t)near_value(g_sensors.fltFL45);
    int16_t frValue = (int16_t)near_value(g_sensors.fltFR45);
    int16_t leftValue = (int16_t)near_value(g_sensors.fltL90);
    int16_t rightValue = (int16_t)near_value(g_sensors.fltR90);

    int16_t flTarget = (int16_t)near_value(g_sensors.targetFL45);
    int16_t frTarget = (int16_t)near_value(g_sensors.targetFR45);
    int16_t leftTarget = (int16_t)near_value(g_sensors.targetL90);
    int16_t rightTarget = (int16_t)near_value(g_sensors.targetR90);

    int16_t leftErr = (int16_t)(
        (SIDE_PID_WEIGHT * (leftValue - leftTarget)) +
        (FRONT_PID_WEIGHT * (flValue - flTarget)));
    int16_t rightErr = (int16_t)(
        (SIDE_PID_WEIGHT * (rightValue - rightTarget)) +
        (FRONT_PID_WEIGHT * (frValue - frTarget)));
    int16_t error = (int16_t)(leftErr - rightErr);
    int16_t frontTurn = (int16_t)(
        (flValue - flTarget) -
        (frValue - frTarget));
    int16_t derivative;
    int32_t correction32;
    int16_t correction;
    int16_t frontAssist = 0;
    int16_t leftSpeed;
    int16_t rightSpeed;

    pidI = clamp_i16((int16_t)(pidI + error), -INTEGRAL_LIMIT, INTEGRAL_LIMIT);
    derivative = (int16_t)(error - lastError);
    lastError = error;

    correction32 =
        ((int32_t)PID_KP * error) +
        ((int32_t)PID_KI * pidI) +
        ((int32_t)PID_KD * derivative);
    correction32 /= PID_SCALE;

    if (correction32 > CORRECTION_LIMIT) {
        correction = CORRECTION_LIMIT;
    } else if (correction32 < -CORRECTION_LIMIT) {
        correction = -CORRECTION_LIMIT;
    } else {
        correction = (int16_t)correction32;
    }

    if (!sensors_wall_front()) {
        if (frontTurn >= FRONT_TURN_DIFF) {
            frontAssist = clamp_i16(
                (int16_t)(frontTurn / FRONT_TURN_ASSIST_SCALE),
                0,
                FRONT_TURN_ASSIST_LIMIT);
            frontAssist = clamp_i16(
                (int16_t)(frontAssist + RIGHT_TURN_EXTRA_ASSIST),
                0,
                FRONT_TURN_ASSIST_LIMIT);
        } else if (frontTurn <= -FRONT_TURN_DIFF) {
            frontAssist = clamp_i16(
                (int16_t)(frontTurn / FRONT_TURN_ASSIST_SCALE),
                -FRONT_TURN_ASSIST_LIMIT,
                0);
        }

        correction = clamp_i16(
            (int16_t)(correction + frontAssist),
            -CORRECTION_LIMIT,
            CORRECTION_LIMIT);
    }

    leftSpeed = clamp_i16((int16_t)(BASE_PWM + correction), 0, PWM_MAX);
    rightSpeed = clamp_i16((int16_t)(BASE_PWM - correction), 0, PWM_MAX);

    motor_set(leftSpeed, rightSpeed);
}

bool turn_alignment_ok(void)
{
    return true;
}

bool turn180_done(void)
{
    return elapsed(turnStartMs) >= TURN_180_MAX_MS;
}

void robot_update(void)
{
    switch (state) {
    case ST_INIT:
        motor_stop();
        if (sensors_ready()) {
            robot_enter(ST_FOLLOW_CORRIDOR);
        }
        break;

    case ST_FOLLOW_CORRIDOR:
        robot_follow_corridor();
        if (sensors_wall_front()) {
            if (!turnbackLogged) {
                UART_PUTS("TURNBACK TRIGGER");
                debug_print_sensor_snapshot();
                UART_PUTS("\r\n");
                turnbackLogged = true;
            }
            motor_stop();
            turnStartMs = millis();
            robot_enter(ST_TURN_BACK);
        }
        break;

    case ST_TURN_BACK:
        motor_set(TURN_BACK_SPEED, -TURN_BACK_SPEED);
        sensors_reset_node_debounce();
        if (turn180_done()) {
            UART_PUTS("TURN B DONE dt=");
            uart_put_u32(elapsed(turnStartMs));
            UART_PUTS("\r\n");
            motor_stop();
            robot_enter(ST_FOLLOW_CORRIDOR);
        }
        break;

    case ST_FINISHED:
        motor_stop();
        break;

    case ST_ERROR:
    default:
        motor_stop();
        robot_enter(ST_ERROR);
        break;
    }
}

void debug_led_init(void)
{
    DDRB |= (uint8_t)(1U << DEBUG_LED);
    PORTB &= (uint8_t)~(1U << DEBUG_LED);
}

void debug_led_update(void)
{
    uint32_t now = millis();
    bool ledOn = false;

    if (state == ST_INIT) {
        ledOn = ((now & 0x0100UL) == 0U);
    } else if (state == ST_ERROR) {
        ledOn = ((now & 0x0200UL) == 0U);
    } else if (state == ST_TURN_BACK) {
        ledOn = ((now & 0x0080UL) == 0U);
    } else if (debugPulseActive) {
        if (elapsed(debugPulseStartMs) < 80UL) {
            ledOn = true;
        } else {
            debugPulseActive = false;
        }
    }

    if (ledOn) {
        PORTB |= (uint8_t)(1U << DEBUG_LED);
    } else {
        PORTB &= (uint8_t)~(1U << DEBUG_LED);
    }
}

RobotState robot_get_state(void)
{
    return state;
}
