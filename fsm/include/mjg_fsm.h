/*
 * mjg_fsm.h - table-driven state machine.
 *
 * SPDX-License-Identifier: MIT
 * Copyright (c) 2021 Maicon Galiazi
 *
 * A machine is a table of { from, condition, to } rows. Each step, the
 * engine takes the first row that leaves the current state and whose
 * condition is true. Conditions are polled, so this suits a super-loop or a
 * periodic callback. Array order is priority order.
 *
 * A state's action is its entry action: it runs once, on entry, in the same
 * step as the transition that got there. README.md has the full contract.
 *
 * Needs only stdbool.h, stdint.h and stddef.h. To use it, copy this file and
 * mjg_fsm.c into your project.
 */

#ifndef MJG_FSM_H
#define MJG_FSM_H

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/* State id. Valid ids are 0 .. (state_count - 1). */
typedef uint8_t mjg_fsm_state;

/* Largest allowed state_count, so the largest valid id is 254. */
#define MJG_FSM_MAX_STATES 255u

/* What mjg_fsm_current() returns for an uninitialised machine. */
#define MJG_FSM_NO_STATE ((mjg_fsm_state)0xFFu)

/* Element count of a fixed-size array. */
#define MJG_FSM_COUNT_OF(arr) (sizeof(arr) / sizeof((arr)[0]))

/* True when the transition may be taken. NULL means always true. */
typedef bool (*mjg_fsm_condition_fn)(void *user);

/* Entry action. A NULL entry means the state has none. */
typedef void (*mjg_fsm_action_fn)(void *user);

typedef struct {
    mjg_fsm_state        from;
    mjg_fsm_condition_fn condition; /* NULL means unconditional */
    mjg_fsm_state        to;
} mjg_fsm_transition;

/* Read-only, so declare it const and let it live in flash. One table can
 * back any number of machines. */
typedef struct {
    const mjg_fsm_action_fn  *actions;          /* by state id; entries may be NULL */
    uint16_t                  state_count;      /* 1 .. MJG_FSM_MAX_STATES          */
    const mjg_fsm_transition *transitions;      /* array order is priority order    */
    uint16_t                  transition_count; /* may be 0                         */
} mjg_fsm_table;

/* Table initialiser that derives both counts from the arrays, so they cannot
 * drift apart:
 *
 *     static const mjg_fsm_table table = MJG_FSM_TABLE(actions, transitions);
 *
 * Both arguments must be arrays in scope, not pointers. */
#define MJG_FSM_TABLE(actions_arr, transitions_arr) \
    {                                               \
        (actions_arr),                              \
        (uint16_t)MJG_FSM_COUNT_OF(actions_arr),    \
        (transitions_arr),                          \
        (uint16_t)MJG_FSM_COUNT_OF(transitions_arr) \
    }

/* One machine. The caller owns the storage. Treat the fields as private and
 * read the state through mjg_fsm_current(). */
struct mjg_fsm {
    const mjg_fsm_table *table;
    void                *user;
    mjg_fsm_state        current;
    bool                 pending_entry;
};
typedef struct mjg_fsm mjg_fsm;

/* Puts the machine in 'initial' with its entry action pending, so the first
 * step runs it. 'user' is passed to every condition and action, and may be
 * NULL. Returns false, leaving the machine inert, if fsm is NULL, the table
 * is invalid, or 'initial' is not a valid id. */
bool mjg_fsm_init(mjg_fsm *fsm, const mjg_fsm_table *table, mjg_fsm_state initial, void *user);

/* Runs any pending entry action, then takes at most one transition. Does
 * nothing if fsm is NULL or was never initialised. */
void mjg_fsm_step(mjg_fsm *fsm);

/* Current state id, or MJG_FSM_NO_STATE if uninitialised. */
mjg_fsm_state mjg_fsm_current(const mjg_fsm *fsm);

/* Rejects the mistakes that would otherwise become a wild jump: a missing or
 * empty action array, an oversized state_count, a NULL transition array with
 * a non-zero count, and any out-of-range 'from' or 'to'. mjg_fsm_init()
 * calls this, so call it directly only to check a table on its own. */
bool mjg_fsm_validate(const mjg_fsm_table *table);

#ifdef __cplusplus
} /* extern "C" */
#endif

#endif /* MJG_FSM_H */
