#include <stdbool.h>
#include <stdint.h>
#include "config.h"
#include "maze_dfs.h"
#include "timer.h"

static NodeState nodeStack[MAX_NODE_STACK];
static uint8_t nodeTop;

static HistoryStep history[MAX_HISTORY];
static uint8_t historyTop;

static RelDir lastMove;
static uint32_t lastNodeMs;
static bool dfsInitialized;
static bool backtrackPending;
static bool finished;
static bool errorFlag;

static void node_init(NodeState *node, uint8_t openMask, RelDir cameFrom)
{
    node->openMask = openMask & BIT_ALL_DIRS;
    node->triedMask = 0;
    node->bannedMask = 0;
    node->cameFrom = (uint8_t)cameFrom;
}

static bool similar_step(HistoryStep a, HistoryStep b)
{
    uint16_t diff;

    if (a.nodeMask != b.nodeMask) {
        return false;
    }
    if (a.move != b.move) {
        return false;
    }

    if (a.travelMs > b.travelMs) {
        diff = (uint16_t)(a.travelMs - b.travelMs);
    } else {
        diff = (uint16_t)(b.travelMs - a.travelMs);
    }

    return diff <= TRAVEL_TIME_TOLERANCE_MS;
}

void dfs_init(void)
{
    uint8_t i;

    for (i = 0; i < MAX_NODE_STACK; i++) {
        node_init(&nodeStack[i], 0, DIR_BACK);
    }

    for (i = 0; i < MAX_HISTORY; i++) {
        history[i].nodeMask = 0;
        history[i].move = DIR_FRONT;
        history[i].travelMs = 0;
    }

    nodeTop = 0;
    historyTop = 0;
    lastMove = DIR_FRONT;
    lastNodeMs = millis();
    dfsInitialized = false;
    backtrackPending = false;
    finished = false;
    errorFlag = false;
}

void dfs_on_node(uint8_t openMask)
{
    uint8_t mask = (openMask | BIT_BACK) & BIT_ALL_DIRS;

    if (!dfsInitialized) {
        nodeTop = 0;
        node_init(&nodeStack[nodeTop], mask, DIR_BACK);
        dfsInitialized = true;
        return;
    }

    if (backtrackPending) {
        backtrackPending = false;
        nodeStack[nodeTop].openMask = mask;
        return;
    }

    if (nodeTop >= (MAX_NODE_STACK - 1U)) {
        errorFlag = true;
        return;
    }

    nodeTop++;
    node_init(&nodeStack[nodeTop], mask, DIR_BACK);
}

RelDir dfs_choose_next_move(void)
{
    NodeState *node = &nodeStack[nodeTop];
    uint8_t available = node->openMask;

    available &= (uint8_t)~node->triedMask;
    available &= (uint8_t)~node->bannedMask;

    if (available & BIT_FRONT) {
        node->triedMask |= BIT_FRONT;
        lastMove = DIR_FRONT;
        return DIR_FRONT;
    }

    if (available & BIT_LEFT) {
        node->triedMask |= BIT_LEFT;
        lastMove = DIR_LEFT;
        return DIR_LEFT;
    }

    if (available & BIT_RIGHT) {
        node->triedMask |= BIT_RIGHT;
        lastMove = DIR_RIGHT;
        return DIR_RIGHT;
    }

    if (nodeTop == 0U) {
        finished = true;
    }

    node->triedMask |= BIT_BACK;
    lastMove = DIR_BACK;
    return DIR_BACK;
}

void dfs_mark_move_tried(RelDir dir)
{
    if (dir <= DIR_BACK) {
        nodeStack[nodeTop].triedMask |= (uint8_t)(1U << dir);
        lastMove = dir;
    }
}

void dfs_ban_move(RelDir dir)
{
    if (dir <= DIR_BACK) {
        nodeStack[nodeTop].bannedMask |= (uint8_t)(1U << dir);
    }
}

void dfs_push_history(uint8_t nodeMask, RelDir move, uint16_t travelMs)
{
    uint8_t i;

    if (historyTop >= MAX_HISTORY) {
        for (i = 1; i < MAX_HISTORY; i++) {
            history[i - 1U] = history[i];
        }
        historyTop = MAX_HISTORY - 1U;
    }

    history[historyTop].nodeMask = nodeMask & BIT_ALL_DIRS;
    history[historyTop].move = (uint8_t)move;
    history[historyTop].travelMs = travelMs;
    historyTop++;
    lastNodeMs = millis();
}

bool dfs_detect_loop(void)
{
    uint8_t len;
    uint8_t i;

    for (len = LOOP_MIN_LEN; len <= LOOP_MAX_LEN; len++) {
        bool match = true;

        if (historyTop < (uint8_t)(len * 2U)) {
            continue;
        }

        for (i = 0; i < len; i++) {
            HistoryStep a = history[historyTop - len + i];
            HistoryStep b = history[historyTop - (uint8_t)(2U * len) + i];

            if (!similar_step(a, b)) {
                match = false;
                break;
            }
        }

        if (match) {
            return true;
        }
    }

    return false;
}

void dfs_backtrack_arrived(void)
{
    if (nodeTop > 0U) {
        nodeTop--;
    } else {
        finished = true;
    }

    backtrackPending = true;
    lastNodeMs = millis();
}

bool dfs_is_finished(void)
{
    return finished;
}

bool dfs_error(void)
{
    return errorFlag;
}

RelDir dfs_get_last_move(void)
{
    return lastMove;
}
