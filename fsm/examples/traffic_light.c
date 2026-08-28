/*
 * traffic_light.c - a worked mjg_fsm machine.
 *
 * SPDX-License-Identifier: MIT
 * Copyright (c) 2021 Maicon Galiazi
 *
 * A traffic light with a pedestrian button, driven by a plain loop. It shows
 * the three things that characterise this style of machine:
 *
 *   - Conditions are polled, once per step. There is no event queue.
 *   - Time is just another input. The engine has no clock, so this example
 *     counts its own ticks; on hardware you would read your own timebase.
 *   - Array order is priority order: in GREEN the button is listed above the
 *     timeout, so a waiting pedestrian cuts the green short.
 *
 * Build:
 *     cc -std=c99 -I../include ../src/mjg_fsm.c traffic_light.c -o traffic_light
 */

#include <stdio.h>

#include "mjg_fsm.h"

/* Durations, in ticks. */
#define RED_TICKS       4u
#define RED_AMBER_TICKS 2u
#define GREEN_TICKS     8u
#define GREEN_MIN_TICKS 3u /* shortest green the button may cut to */
#define AMBER_TICKS     2u

enum {
    ST_RED,
    ST_RED_AMBER,
    ST_GREEN,
    ST_AMBER
};

typedef struct {
    unsigned tick;       /* advanced by the loop below   */
    unsigned entered_at; /* tick the current state began */
    bool     button;     /* pedestrian request, pending  */
} crossing;

static bool held_for(const crossing *c, unsigned ticks)
{
    return (c->tick - c->entered_at) >= ticks;
}

/* -- entry actions ---------------------------------------------------- */

static void notify(crossing *c, const char *lamps)
{
    c->entered_at = c->tick;
    printf("  t=%2u  %s\n", c->tick, lamps);
}

static void enter_red(void *user)
{
    crossing *c = (crossing *)user;

    c->button = false; /* arriving at red serves the request */
    notify(c, "RED");
}

static void enter_red_amber(void *user)
{
    notify((crossing *)user, "RED + AMBER");
}

static void enter_green(void *user)
{
    notify((crossing *)user, "GREEN");
}

static void enter_amber(void *user)
{
    notify((crossing *)user, "AMBER");
}

/* -- conditions ------------------------------------------------------- */

static bool red_expired(void *user)
{
    return held_for((crossing *)user, RED_TICKS);
}

static bool red_amber_expired(void *user)
{
    return held_for((crossing *)user, RED_AMBER_TICKS);
}

static bool pedestrian_waiting(void *user)
{
    const crossing *c = (const crossing *)user;

    return c->button && held_for(c, GREEN_MIN_TICKS);
}

static bool green_expired(void *user)
{
    return held_for((crossing *)user, GREEN_TICKS);
}

static bool amber_expired(void *user)
{
    return held_for((crossing *)user, AMBER_TICKS);
}

/* -- the table -------------------------------------------------------- */

static const mjg_fsm_action_fn actions[] = {
    enter_red,       /* ST_RED       */
    enter_red_amber, /* ST_RED_AMBER */
    enter_green,     /* ST_GREEN     */
    enter_amber      /* ST_AMBER     */
};

static const mjg_fsm_transition transitions[] = {
    { ST_RED,       red_expired,        ST_RED_AMBER },
    { ST_RED_AMBER, red_amber_expired,  ST_GREEN     },

    /* The button is checked before the timeout, so it wins. */
    { ST_GREEN,     pedestrian_waiting, ST_AMBER     },
    { ST_GREEN,     green_expired,      ST_AMBER     },

    { ST_AMBER,     amber_expired,      ST_RED       }
};

static const mjg_fsm_table table = MJG_FSM_TABLE(actions, transitions);

int main(void)
{
    crossing c;
    mjg_fsm  light;
    unsigned i;

    c.tick       = 0u;
    c.entered_at = 0u;
    c.button     = false;

    if (!mjg_fsm_init(&light, &table, ST_RED, &c)) {
        printf("the table is malformed\n");
        return 1;
    }

    printf("traffic light: 30 ticks, button pressed at t=10\n\n");

    for (i = 0u; i < 30u; ++i) {
        if (c.tick == 10u) {
            printf("  t=%2u  [button pressed]\n", c.tick);
            c.button = true;
        }

        mjg_fsm_step(&light);
        c.tick++;
    }

    return 0;
}
