/*
 * mjg_fsm.c - table-driven state machine.
 *
 * SPDX-License-Identifier: MIT
 * Copyright (c) 2021 Maicon Galiazi
 *
 * See mjg_fsm.h for the contract.
 */

#include "mjg_fsm.h"

/* 'state' is always in range: init checks the initial id, and validate
 * checks every 'to'. */
static void run_entry_action(const mjg_fsm_table *table, mjg_fsm_state state, void *user)
{
    mjg_fsm_action_fn action = table->actions[state];

    if (action != NULL) {
        action(user);
    }
}

bool mjg_fsm_validate(const mjg_fsm_table *table)
{
    uint16_t i;

    if (table == NULL) {
        return false;
    }
    if (table->state_count == 0u || table->state_count > MJG_FSM_MAX_STATES) {
        return false;
    }
    if (table->actions == NULL) {
        return false;
    }
    if (table->transition_count > 0u && table->transitions == NULL) {
        return false;
    }

    for (i = 0u; i < table->transition_count; ++i) {
        if (table->transitions[i].from >= table->state_count) {
            return false;
        }
        if (table->transitions[i].to >= table->state_count) {
            return false;
        }
    }

    return true;
}

bool mjg_fsm_init(mjg_fsm *fsm, const mjg_fsm_table *table, mjg_fsm_state initial, void *user)
{
    if (fsm == NULL) {
        return false;
    }

    /* Stay inert on failure, so an ignored return value shows up as nothing
     * happening rather than as state 0 running. */
    fsm->table         = NULL;
    fsm->user          = NULL;
    fsm->current       = MJG_FSM_NO_STATE;
    fsm->pending_entry = false;

    if (!mjg_fsm_validate(table) || initial >= table->state_count) {
        return false;
    }

    fsm->table         = table;
    fsm->user          = user;
    fsm->current       = initial;
    fsm->pending_entry = true;

    return true;
}

void mjg_fsm_step(mjg_fsm *fsm)
{
    const mjg_fsm_table *table;
    uint16_t             i;

    if (fsm == NULL || fsm->table == NULL) {
        return;
    }
    table = fsm->table;

    /* Only the initial state arrives here pending; later entry actions run
     * inline below. */
    if (fsm->pending_entry) {
        fsm->pending_entry = false;
        run_entry_action(table, fsm->current, fsm->user);
    }

    for (i = 0u; i < table->transition_count; ++i) {
        const mjg_fsm_transition *t = &table->transitions[i];

        if (t->from != fsm->current) {
            continue;
        }
        if (t->condition != NULL && !t->condition(fsm->user)) {
            continue;
        }

        /* First match wins, and it is the only one taken this step. A
         * self-transition lands here too, which is what re-runs its action. */
        fsm->current = t->to;
        run_entry_action(table, fsm->current, fsm->user);
        return;
    }
}

mjg_fsm_state mjg_fsm_current(const mjg_fsm *fsm)
{
    if (fsm == NULL) {
        return MJG_FSM_NO_STATE;
    }
    return fsm->current;
}
