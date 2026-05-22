#ifndef ROBOT_H
#define ROBOT_H

#include <stdbool.h>
#include "config.h"

typedef enum {
    ST_INIT = 0,

    ST_FOLLOW_CORRIDOR,
    ST_NODE_DETECTED,
    ST_DECIDE_MOVE,

    ST_TURN_LEFT,
    ST_TURN_RIGHT,
    ST_TURN_BACK,

    ST_LEAVE_NODE,
    ST_MOVE_TO_NEXT_NODE,

    ST_BACKTRACK,
    ST_FINISHED,
    ST_ERROR
} RobotState;

void robot_init(void);
void robot_update(void);
void robot_follow_corridor(void);
bool turn180_done(void);
bool turn_alignment_ok(void);

void debug_led_init(void);
void debug_led_update(void);
RobotState robot_get_state(void);

#endif
