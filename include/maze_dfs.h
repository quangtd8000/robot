#ifndef MAZE_DFS_H
#define MAZE_DFS_H

#include <stdbool.h>
#include <stdint.h>
#include "config.h"

typedef struct {
    uint8_t openMask;
    uint8_t triedMask;
    uint8_t bannedMask;
    uint8_t cameFrom;
} NodeState;

typedef struct {
    uint8_t nodeMask;
    uint8_t move;
    uint16_t travelMs;
} HistoryStep;

void dfs_init(void);
void dfs_on_node(uint8_t openMask);
RelDir dfs_choose_next_move(void);
void dfs_mark_move_tried(RelDir dir);
void dfs_ban_move(RelDir dir);
void dfs_push_history(uint8_t nodeMask, RelDir move, uint16_t travelMs);
bool dfs_detect_loop(void);
void dfs_backtrack_arrived(void);
bool dfs_is_finished(void);
bool dfs_error(void);
RelDir dfs_get_last_move(void);

#endif
