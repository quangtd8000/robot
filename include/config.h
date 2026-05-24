#ifndef CONFIG_H
#define CONFIG_H

#ifndef F_CPU
#define F_CPU 1000000UL
#endif

#include <avr/io.h>

#define UART_BAUD 9600UL

#define PWM_MAX 255
#define BASE_PWM 140
#define BASE_SPEED BASE_PWM
#define TURN_BACK_SPEED 130
#define TURN_LEFT_LOW_PWM 130
#define TURN_LEFT_HIGH_PWM 195
#define TURN_RIGHT_LOW_PWM 120
#define TURN_RIGHT_HIGH_PWM 255

/* Measured motor start PWM. Non-zero commands below this are lifted here. */
#define L_MOTOR_MIN_PWM 50
#define R_MOTOR_MIN_PWM 60

/*
 * ADC mapping.
 *
 * ADC pin map:
 *   ADC0 / PC0  = S_FL45
 *   ADC1 / PC1  = S_FR45
 *   ADC2 / PC2  = S_REAR
 *   ADC6        = S_R90
 *   ADC7        = S_L90
 *
 * ADC6/ADC7 are analog-only inputs on ATmega328P packages that expose them.
 * If your board instead wires side sensors to PC3/PC4, change ADC_R90 to 3
 * and ADC_L90 to 4.
 */
#define ADC_FL45 0
#define ADC_FR45 1
#define ADC_REAR 2
#define ADC_R90 6
#define ADC_L90 7
#define ADC_AVG_SAMPLES 10

#define L_PWM_PIN PB1
#define R_PWM_PIN PB2

/*
 * Motor direction mapping.
 *
 * Defaults follow the wiring description:
 *   left direction  = PD7, L_IN2 tied to GND
 *   right direction = PB0, R_IN2 tied to GND
 *
 * Set MOTOR_IN2_TIED_GND to 0 and adjust the *_IN2_* macros if using a
 * two-input H-bridge instead.
 */
#define MOTOR_IN2_TIED_GND 1

/*
 * From motor ramp test:
 *   left physical forward  = PD7 low
 *   right physical forward = PB0 low
 */
#define L_DIR_HIGH_FORWARD 0
#define R_DIR_HIGH_FORWARD 0

#define L_IN1 PD7
#define L_IN2 PD3
#define R_IN1 PB0
#define R_IN2 PD4

#define L_IN1_DDR DDRD
#define L_IN1_PORT PORTD
#define L_IN2_DDR DDRD
#define L_IN2_PORT PORTD

#define R_IN1_DDR DDRB
#define R_IN1_PORT PORTB
#define R_IN2_DDR DDRD
#define R_IN2_PORT PORTD

#define DEBUG_LED PB5

#define START_BUTTON_PIN PD2
#define START_BUTTON_DDR DDRD
#define START_BUTTON_PORT PORTD
#define START_BUTTON_INPUT PIND

/* If the sensor ADC value increases when an object is nearer, keep 1. */
#define SENSOR_NEAR_IS_HIGH 1

/* Initial thresholds. Tune on the real maze. */
#define WALL_L_THRESHOLD 180
#define WALL_R_THRESHOLD 185
#define WALL_FRONT_THRESHOLD 90
#define WALL_REAR_THRESHOLD 170
#define WALL_HYSTERESIS 40
#define SIDE_WALL_THRESHOLD_OFFSET 80

/*
 * Startup ADC calibration:
 * keep the robot still for SENSOR_CALIBRATION_MS after reset. The averaged
 * ADC values become runtime thresholds/targets. Put the robot in a normal
 * corridor during this phase: side walls present, front path open.
 */
#define SENSOR_CALIBRATION_MS 1000UL
#define START_DELAY_MS 2000UL
#define STARTUP_SIDE_WALLS_PRESENT 1
#define STARTUP_FRONT_WALL_PRESENT 0
#define STARTUP_REAR_WALL_PRESENT 0

#define FILTER_SHIFT 0

#define PID_KP 12
#define PID_KI 0
#define PID_KD 6
#define PID_SCALE 8
#define INTEGRAL_LIMIT 300
#define CORRECTION_LIMIT 55
#define SIDE_PID_WEIGHT 1
#define FRONT_PID_WEIGHT 1

#define FRONT_TURN_DIFF 30
#define FRONT_TURN_ASSIST_SCALE 2
#define FRONT_TURN_ASSIST_LIMIT 35
#define RIGHT_TURN_EXTRA_ASSIST 8

#define TURNBACK_FRONT_MULT_NUM 9
#define TURNBACK_FRONT_MULT_DEN 5
#define TURNBACK_FRONT_MIN_THRESHOLD WALL_FRONT_THRESHOLD

#define CONTROL_DT_MS 20UL
#define TURN_ALIGN_DIFF_THRESHOLD 60

#define NODE_DEBOUNCE_MS 120UL
#define LEAVE_NODE_MS 800UL

/* Initial values for 1 MHz clock. Tune after measuring real turn rate. */
#define TURN_LEFT_MS 4000UL
#define TURN_RIGHT_MS 3900UL
#define TURN_180_MIN_MS 3500UL
#define TURN_180_MAX_MS 3500UL

#define MAX_TRAVEL_MS 30000UL

#define MAX_NODE_STACK 64
#define MAX_HISTORY 96

#define LOOP_MIN_LEN 2
#define LOOP_MAX_LEN 8
#define TRAVEL_TIME_TOLERANCE_MS 400

typedef enum {
  DIR_LEFT = 0,
  DIR_FRONT = 1,
  DIR_RIGHT = 2,
  DIR_BACK = 3
} RelDir;

#define BIT_LEFT (1U << DIR_LEFT)
#define BIT_FRONT (1U << DIR_FRONT)
#define BIT_RIGHT (1U << DIR_RIGHT)
#define BIT_BACK (1U << DIR_BACK)
#define BIT_ALL_DIRS (BIT_LEFT | BIT_FRONT | BIT_RIGHT | BIT_BACK)

#endif
