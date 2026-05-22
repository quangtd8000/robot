#ifndef SENSORS_H
#define SENSORS_H

#include <stdbool.h>
#include <stdint.h>
#include "config.h"

typedef struct {
    uint16_t rawFL45;
    uint16_t rawFR45;
    uint16_t rawL90;
    uint16_t rawR90;
    uint16_t rawRear;

    uint16_t fltFL45;
    uint16_t fltFR45;
    uint16_t fltL90;
    uint16_t fltR90;
    uint16_t fltRear;

    bool wallLeft;
    bool wallRight;
    bool wallFront;
    bool wallRear;

    bool nodeRaw;
    bool nodeConfirmed;

    uint32_t nodeStartMs;

    uint16_t wallLeftThreshold;
    uint16_t wallRightThreshold;
    uint16_t wallFrontThreshold;
    uint16_t wallRearThreshold;

    uint16_t targetFL45;
    uint16_t targetFR45;
    uint16_t targetL90;
    uint16_t targetR90;

    bool calibrated;
} SensorState;

extern SensorState g_sensors;

void sensors_init(void);
void sensors_update(void);
void sensors_reset_node_debounce(void);
void sensors_set_debug_enabled(bool enabled);
bool sensors_wall_left(void);
bool sensors_wall_right(void);
bool sensors_wall_front(void);
bool sensors_wall_rear(void);
bool sensors_node_confirmed(void);
bool sensors_ready(void);
uint16_t sensors_wall_left_threshold(void);
uint16_t sensors_wall_right_threshold(void);
uint16_t sensors_wall_front_threshold(void);
uint16_t sensors_wall_rear_threshold(void);
uint8_t sensors_get_open_mask(void);

#endif
