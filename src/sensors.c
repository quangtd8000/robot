#include <stdbool.h>
#include <stdint.h>
#include "adc.h"
#include "config.h"
#include "sensors.h"
#include "timer.h"
#include "uart.h"

SensorState g_sensors;

static uint32_t calibStartMs;
static uint32_t calibCount;
static uint32_t sumFL45;
static uint32_t sumFR45;
static uint32_t sumL90;
static uint32_t sumR90;
static uint32_t sumRear;
static bool debugEnabled = true;

static uint16_t ema_update(uint16_t filtered, uint16_t raw)
{
#if FILTER_SHIFT == 0
    (void)filtered;
    return raw;
#else
    int32_t diff = (int32_t)raw - (int32_t)filtered;
    return (uint16_t)((int32_t)filtered + (diff >> FILTER_SHIFT));
#endif
}

#if !SENSOR_NEAR_IS_HIGH
static uint16_t near_value(uint16_t value)
{
    return (uint16_t)(1023U - value);
}
#endif

static bool wall_hysteresis(uint16_t value, uint16_t threshold, bool wasWall)
{
    int16_t lower = (int16_t)threshold - (int16_t)WALL_HYSTERESIS;
    int16_t upper = (int16_t)threshold + (int16_t)WALL_HYSTERESIS;
    int16_t sample = (int16_t)value;

#if SENSOR_NEAR_IS_HIGH
    if (wasWall) {
        return sample > lower;
    }
    return sample > upper;
#else
    if (wasWall) {
        return sample < upper;
    }
    return sample < lower;
#endif
}

static bool front_wall_detected(void)
{
    uint16_t flThreshold;
    uint16_t frThreshold;

#if SENSOR_NEAR_IS_HIGH
    flThreshold = (uint16_t)(((uint32_t)g_sensors.targetFL45 *
        TURNBACK_FRONT_MULT_NUM) / TURNBACK_FRONT_MULT_DEN);
    frThreshold = (uint16_t)(((uint32_t)g_sensors.targetFR45 *
        TURNBACK_FRONT_MULT_NUM) / TURNBACK_FRONT_MULT_DEN);

    if (flThreshold < TURNBACK_FRONT_MIN_THRESHOLD) {
        flThreshold = TURNBACK_FRONT_MIN_THRESHOLD;
    }
    if (frThreshold < TURNBACK_FRONT_MIN_THRESHOLD) {
        frThreshold = TURNBACK_FRONT_MIN_THRESHOLD;
    }

    return (g_sensors.fltFL45 >= flThreshold) &&
        (g_sensors.fltFR45 >= frThreshold);
#else
    flThreshold = (uint16_t)(1023U -
        (((uint32_t)near_value(g_sensors.targetFL45) *
          TURNBACK_FRONT_MULT_NUM) / TURNBACK_FRONT_MULT_DEN));
    frThreshold = (uint16_t)(1023U -
        (((uint32_t)near_value(g_sensors.targetFR45) *
          TURNBACK_FRONT_MULT_NUM) / TURNBACK_FRONT_MULT_DEN));

    return (g_sensors.fltFL45 <= flThreshold) &&
        (g_sensors.fltFR45 <= frThreshold);
#endif
}

static uint16_t side_wall_threshold_from_calibration(uint16_t value)
{
#if SENSOR_NEAR_IS_HIGH
    if (value > SIDE_WALL_THRESHOLD_OFFSET) {
        return (uint16_t)(value - SIDE_WALL_THRESHOLD_OFFSET);
    }
    return 0;
#else
    if (value < (1023U - SIDE_WALL_THRESHOLD_OFFSET)) {
        return (uint16_t)(value + SIDE_WALL_THRESHOLD_OFFSET);
    }
    return 1023U;
#endif
}

static void debug_print_sensor_values(void)
{
    UART_PUTS(" ADC FL=");
    uart_put_u16(g_sensors.fltFL45);
    UART_PUTS(" FR=");
    uart_put_u16(g_sensors.fltFR45);
    UART_PUTS(" L90=");
    uart_put_u16(g_sensors.fltL90);
    UART_PUTS(" R90=");
    uart_put_u16(g_sensors.fltR90);
    UART_PUTS(" RE=");
    uart_put_u16(g_sensors.fltRear);
}

static void debug_print_thresholds(void)
{
    UART_PUTS(" TH L=");
    uart_put_u16(g_sensors.wallLeftThreshold);
    UART_PUTS(" R=");
    uart_put_u16(g_sensors.wallRightThreshold);
    UART_PUTS(" F=");
    uart_put_u16(g_sensors.wallFrontThreshold);
    UART_PUTS(" RE=");
    uart_put_u16(g_sensors.wallRearThreshold);
    UART_PUTS(" TG FL=");
    uart_put_u16(g_sensors.targetFL45);
    UART_PUTS(" FR=");
    uart_put_u16(g_sensors.targetFR45);
    UART_PUTS(" L=");
    uart_put_u16(g_sensors.targetL90);
    UART_PUTS(" R=");
    uart_put_u16(g_sensors.targetR90);
}

static void debug_print_walls(void)
{
    UART_PUTS(" W L=");
    uart_put_u8(g_sensors.wallLeft ? 1U : 0U);
    UART_PUTS(" F=");
    uart_put_u8(g_sensors.wallFront ? 1U : 0U);
    UART_PUTS(" R=");
    uart_put_u8(g_sensors.wallRight ? 1U : 0U);
    UART_PUTS(" RE=");
    uart_put_u8(g_sensors.wallRear ? 1U : 0U);
}

static void debug_log_wall_change(void)
{
    UART_PUTS("WALL");
    debug_print_walls();
    debug_print_sensor_values();
    debug_print_thresholds();
    UART_PUTS("\r\n");
}

static void update_raw(void)
{
    g_sensors.rawFL45 = adc_read_avg(ADC_FL45);
    g_sensors.rawFR45 = adc_read_avg(ADC_FR45);
    g_sensors.rawL90 = adc_read_avg(ADC_L90);
    g_sensors.rawR90 = adc_read_avg(ADC_R90);
    g_sensors.rawRear = adc_read_avg(ADC_REAR);
}

static void update_filtered(void)
{
    g_sensors.fltFL45 = ema_update(g_sensors.fltFL45, g_sensors.rawFL45);
    g_sensors.fltFR45 = ema_update(g_sensors.fltFR45, g_sensors.rawFR45);
    g_sensors.fltL90 = ema_update(g_sensors.fltL90, g_sensors.rawL90);
    g_sensors.fltR90 = ema_update(g_sensors.fltR90, g_sensors.rawR90);
    g_sensors.fltRear = ema_update(g_sensors.fltRear, g_sensors.rawRear);
}

static void update_walls(void)
{
    g_sensors.wallLeft = wall_hysteresis(
        g_sensors.fltL90,
        g_sensors.wallLeftThreshold,
        g_sensors.wallLeft);

    g_sensors.wallRight = wall_hysteresis(
        g_sensors.fltR90,
        g_sensors.wallRightThreshold,
        g_sensors.wallRight);

    g_sensors.wallFront = front_wall_detected();

    g_sensors.wallRear = wall_hysteresis(
        g_sensors.fltRear,
        g_sensors.wallRearThreshold,
        g_sensors.wallRear);
}

static void update_node_debounce(void)
{
    g_sensors.nodeStartMs = 0;
    g_sensors.nodeRaw = false;
    g_sensors.nodeConfirmed = false;
}

void sensors_reset_node_debounce(void)
{
    g_sensors.nodeStartMs = 0;
    g_sensors.nodeRaw = false;
    g_sensors.nodeConfirmed = false;
}

void sensors_set_debug_enabled(bool enabled)
{
    debugEnabled = enabled;
}

static void calibration_reset(void)
{
    calibStartMs = millis();
    calibCount = 0;
    sumFL45 = 0;
    sumFR45 = 0;
    sumL90 = 0;
    sumR90 = 0;
    sumRear = 0;

    g_sensors.wallLeftThreshold = WALL_L_THRESHOLD;
    g_sensors.wallRightThreshold = WALL_R_THRESHOLD;
    g_sensors.wallFrontThreshold = WALL_FRONT_THRESHOLD;
    g_sensors.wallRearThreshold = WALL_REAR_THRESHOLD;
    g_sensors.targetFL45 = WALL_FRONT_THRESHOLD;
    g_sensors.targetFR45 = WALL_FRONT_THRESHOLD;
    g_sensors.targetL90 = WALL_L_THRESHOLD;
    g_sensors.targetR90 = WALL_R_THRESHOLD;
    g_sensors.calibrated = false;
}

static void calibration_accumulate(void)
{
    sumFL45 += g_sensors.rawFL45;
    sumFR45 += g_sensors.rawFR45;
    sumL90 += g_sensors.rawL90;
    sumR90 += g_sensors.rawR90;
    sumRear += g_sensors.rawRear;
    calibCount++;
}

static void calibration_finish(void)
{
    uint16_t avgFL45;
    uint16_t avgFR45;

    if (calibCount == 0U) {
        g_sensors.calibrated = true;
        return;
    }

    avgFL45 = (uint16_t)(sumFL45 / calibCount);
    avgFR45 = (uint16_t)(sumFR45 / calibCount);

    g_sensors.fltFL45 = avgFL45;
    g_sensors.fltFR45 = avgFR45;
    g_sensors.fltL90 = (uint16_t)(sumL90 / calibCount);
    g_sensors.fltR90 = (uint16_t)(sumR90 / calibCount);
    g_sensors.fltRear = (uint16_t)(sumRear / calibCount);

    g_sensors.targetFL45 = g_sensors.fltFL45;
    g_sensors.targetFR45 = g_sensors.fltFR45;
    g_sensors.targetL90 = g_sensors.fltL90;
    g_sensors.targetR90 = g_sensors.fltR90;

    g_sensors.wallLeftThreshold =
        side_wall_threshold_from_calibration(g_sensors.fltL90);
    g_sensors.wallRightThreshold =
        side_wall_threshold_from_calibration(g_sensors.fltR90);
    g_sensors.wallFrontThreshold = WALL_FRONT_THRESHOLD;
    g_sensors.wallRearThreshold = g_sensors.fltRear;

    g_sensors.wallLeft = (STARTUP_SIDE_WALLS_PRESENT != 0);
    g_sensors.wallRight = (STARTUP_SIDE_WALLS_PRESENT != 0);
    g_sensors.wallFront = (STARTUP_FRONT_WALL_PRESENT != 0);
    g_sensors.wallRear = (STARTUP_REAR_WALL_PRESENT != 0);
    sensors_reset_node_debounce();
    g_sensors.calibrated = true;

    UART_PUTS("CAL DONE");
    debug_print_sensor_values();
    debug_print_thresholds();
    UART_PUTS("\r\n");
}

void sensors_init(void)
{
    update_raw();

    g_sensors.fltFL45 = g_sensors.rawFL45;
    g_sensors.fltFR45 = g_sensors.rawFR45;
    g_sensors.fltL90 = g_sensors.rawL90;
    g_sensors.fltR90 = g_sensors.rawR90;
    g_sensors.fltRear = g_sensors.rawRear;

    calibration_reset();
}

void sensors_update(void)
{
    update_raw();

    if (!g_sensors.calibrated) {
        calibration_accumulate();
        g_sensors.fltFL45 = g_sensors.rawFL45;
        g_sensors.fltFR45 = g_sensors.rawFR45;
        g_sensors.fltL90 = g_sensors.rawL90;
        g_sensors.fltR90 = g_sensors.rawR90;
        g_sensors.fltRear = g_sensors.rawRear;
        sensors_reset_node_debounce();

        if (elapsed(calibStartMs) >= SENSOR_CALIBRATION_MS) {
            calibration_finish();
        }
        return;
    }

    update_filtered();

    bool oldLeft = g_sensors.wallLeft;
    bool oldRight = g_sensors.wallRight;
    bool oldFront = g_sensors.wallFront;
    bool oldRear = g_sensors.wallRear;
    bool oldNode = g_sensors.nodeConfirmed;

    update_walls();
    update_node_debounce();

    if (debugEnabled &&
        ((oldLeft != g_sensors.wallLeft) ||
         (oldRight != g_sensors.wallRight) ||
         (oldFront != g_sensors.wallFront) ||
         (oldRear != g_sensors.wallRear))) {
        debug_log_wall_change();
    }

    if (debugEnabled && (oldNode != g_sensors.nodeConfirmed)) {
        UART_PUTS("NODE=");
        uart_put_u8(g_sensors.nodeConfirmed ? 1U : 0U);
        debug_print_walls();
        UART_PUTS("\r\n");
    }
}

bool sensors_wall_left(void)
{
    return g_sensors.wallLeft;
}

bool sensors_wall_right(void)
{
    return g_sensors.wallRight;
}

bool sensors_wall_front(void)
{
    return g_sensors.wallFront;
}

bool sensors_wall_rear(void)
{
    return g_sensors.wallRear;
}

bool sensors_node_confirmed(void)
{
    return g_sensors.nodeConfirmed;
}

bool sensors_ready(void)
{
    return g_sensors.calibrated;
}

uint16_t sensors_wall_left_threshold(void)
{
    return g_sensors.wallLeftThreshold;
}

uint16_t sensors_wall_right_threshold(void)
{
    return g_sensors.wallRightThreshold;
}

uint16_t sensors_wall_front_threshold(void)
{
    return g_sensors.wallFrontThreshold;
}

uint16_t sensors_wall_rear_threshold(void)
{
    return g_sensors.wallRearThreshold;
}

uint8_t sensors_get_open_mask(void)
{
    uint8_t mask = BIT_BACK;

    if (!g_sensors.wallLeft) {
        mask |= BIT_LEFT;
    }
    if (!g_sensors.wallFront) {
        mask |= BIT_FRONT;
    }
    if (!g_sensors.wallRight) {
        mask |= BIT_RIGHT;
    }

    return mask;
}
