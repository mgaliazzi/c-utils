/*
 * test_fsm.c - host tests for mjg_fsm.
 *
 * SPDX-License-Identifier: MIT
 * Copyright (c) 2021 Maicon Galiazi
 *
 * No framework: the component must test with nothing but a C99 compiler.
 */

#include <stdio.h>

#include "mjg_fsm.h"

static int checks   = 0;
static int failures = 0;

#define CHECK(cond, msg)                                                \
    do {                                                                \
        ++checks;                                                       \
        if (!(cond)) {                                                  \
            ++failures;                                                 \
            printf("FAIL  %s:%d  %s\n", __FILE__, __LINE__, (msg));     \
        }                                                               \
    } while (0)

/* Every condition and action receives this through 'user'. */
typedef struct {
    int   entered[4]; /* entry count, per state id          */
    int   polls;      /* times the condition was asked      */
    bool  open;       /* what the condition answers         */
    void *seen;       /* the user pointer, as an action saw it */
} fix;

static void reset(fix *x)
{
    static const fix zero = { { 0, 0, 0, 0 }, 0, false, NULL };
    *x = zero;
}

static void enter_0(void *u)
{
    fix *x = (fix *)u;
    x->entered[0]++;
    x->seen = u;
}

static void enter_1(void *u) { ((fix *)u)->entered[1]++; }
static void enter_2(void *u) { ((fix *)u)->entered[2]++; }
static void enter_3(void *u) { ((fix *)u)->entered[3]++; }

static bool is_open(void *u)
{
    fix *x = (fix *)u;
    x->polls++;
    return x->open;
}

/* Shared by every machine below, so state N counts into entered[N]. */
static const mjg_fsm_action_fn actions[] = { enter_0, enter_1, enter_2, enter_3 };

/* 0 -> 1 -> 2, both hops behind the condition. */
static const mjg_fsm_transition gated[] = {
    { 0, is_open, 1 },
    { 1, is_open, 2 },
};
static const mjg_fsm_table gated_table = MJG_FSM_TABLE(actions, gated);

/* Two unconditional ways out of state 0. */
static const mjg_fsm_transition rival[] = {
    { 0, NULL, 1 },
    { 0, NULL, 2 },
};
static const mjg_fsm_table rival_table = MJG_FSM_TABLE(actions, rival);

/* An unconditional self-transition. */
static const mjg_fsm_transition loop[]      = { { 0, NULL, 0 } };
static const mjg_fsm_table      loop_table  = MJG_FSM_TABLE(actions, loop);

/* An unconditional chain. */
static const mjg_fsm_transition chain[] = {
    { 0, NULL, 1 },
    { 1, NULL, 2 },
    { 2, NULL, 3 },
};
static const mjg_fsm_table chain_table = MJG_FSM_TABLE(actions, chain);

/* A machine with no actions at all. */
static const mjg_fsm_action_fn  no_actions[]  = { NULL, NULL };
static const mjg_fsm_transition hop[]         = { { 0, NULL, 1 } };
static const mjg_fsm_table      quiet_table   = MJG_FSM_TABLE(no_actions, hop);

static void test_entry_and_progress(void)
{
    fix     f;
    mjg_fsm m;

    reset(&f);
    CHECK(mjg_fsm_init(&m, &gated_table, 0, &f), "init succeeds");
    CHECK(mjg_fsm_current(&m) == 0, "starts in state 0");
    CHECK(f.entered[0] == 0, "init does not run the action itself");

    mjg_fsm_step(&m);
    CHECK(f.entered[0] == 1, "first step runs the initial entry action");
    CHECK(mjg_fsm_current(&m) == 0, "condition false, so no move");
    CHECK(f.seen == (void *)&f, "the user pointer reaches actions unchanged");
    CHECK(f.polls == 1, "the condition is asked once per step");

    mjg_fsm_step(&m);
    CHECK(f.entered[0] == 1, "the entry action does not repeat while held");

    /* The point of the design: entry lands in the same step as the move. */
    f.open = true;
    mjg_fsm_step(&m);
    CHECK(mjg_fsm_current(&m) == 1, "condition true, so it moves");
    CHECK(f.entered[1] == 1, "entry action runs in the SAME step as the move");
    CHECK(f.entered[0] == 1, "leaving a state does not re-run its action");

    mjg_fsm_step(&m);
    CHECK(mjg_fsm_current(&m) == 2, "and moves again");

    mjg_fsm_step(&m);
    CHECK(mjg_fsm_current(&m) == 2, "nothing leaves state 2, so it rests");
    CHECK(f.entered[2] == 1, "and nothing re-runs");
}

static void test_machines_are_independent(void)
{
    fix     fa, fb;
    mjg_fsm a, b;

    reset(&fa);
    reset(&fb);
    CHECK(mjg_fsm_init(&a, &gated_table, 0, &fa), "a inits");
    CHECK(mjg_fsm_init(&b, &gated_table, 0, &fb), "b inits");

    fa.open = true;
    mjg_fsm_step(&a);
    mjg_fsm_step(&a);
    mjg_fsm_step(&b);

    CHECK(mjg_fsm_current(&a) == 2, "a advanced twice");
    CHECK(mjg_fsm_current(&b) == 0, "b did not move");
    CHECK(fb.entered[1] == 0, "and none of b's later actions ran");
}

static void test_first_match_wins(void)
{
    fix     f;
    mjg_fsm m;

    reset(&f);
    CHECK(mjg_fsm_init(&m, &rival_table, 0, &f), "init");
    mjg_fsm_step(&m);
    CHECK(mjg_fsm_current(&m) == 1, "the earlier row in the array wins");
}

static void test_null_actions_are_skipped(void)
{
    mjg_fsm m;

    CHECK(mjg_fsm_init(&m, &quiet_table, 0, NULL), "init with a NULL user pointer");
    mjg_fsm_step(&m);
    CHECK(mjg_fsm_current(&m) == 1, "NULL actions are skipped, not called");
}

static void test_self_transition_repeats(void)
{
    fix     f;
    mjg_fsm m;

    reset(&f);
    CHECK(mjg_fsm_init(&m, &loop_table, 0, &f), "init");

    mjg_fsm_step(&m);
    CHECK(f.entered[0] == 2, "initial entry, then one re-entry");

    mjg_fsm_step(&m);
    CHECK(f.entered[0] == 3, "one re-entry per step after that");
    CHECK(mjg_fsm_current(&m) == 0, "the state itself never changes");
}

static void test_one_transition_per_step(void)
{
    fix     f;
    mjg_fsm m;

    reset(&f);
    CHECK(mjg_fsm_init(&m, &chain_table, 0, &f), "init");

    mjg_fsm_step(&m);
    CHECK(mjg_fsm_current(&m) == 1, "one hop, not run-to-completion");

    mjg_fsm_step(&m);
    mjg_fsm_step(&m);
    CHECK(mjg_fsm_current(&m) == 3, "one hop per step to the end of the chain");

    mjg_fsm_step(&m);
    CHECK(mjg_fsm_current(&m) == 3, "then it rests");
}

static void test_validate(void)
{
    static const mjg_fsm_action_fn  acts[]     = { NULL, NULL };
    static const mjg_fsm_transition ok[]       = { { 0, NULL, 1 } };
    static const mjg_fsm_transition bad_from[] = { { 9, NULL, 0 } };
    static const mjg_fsm_transition bad_to[]   = { { 0, NULL, 9 } };

    mjg_fsm_table t;

    CHECK(!mjg_fsm_validate(NULL), "a NULL table is invalid");

    t.actions          = acts;
    t.state_count      = 2;
    t.transitions      = ok;
    t.transition_count = 1;
    CHECK(mjg_fsm_validate(&t), "the control case is valid");

    t.transitions = bad_from;
    CHECK(!mjg_fsm_validate(&t), "out-of-range 'from'");

    t.transitions = bad_to;
    CHECK(!mjg_fsm_validate(&t), "out-of-range 'to'");

    t.transitions = ok;
    t.state_count = 0;
    CHECK(!mjg_fsm_validate(&t), "zero state_count");

    t.state_count = MJG_FSM_MAX_STATES + 1u;
    CHECK(!mjg_fsm_validate(&t), "oversized state_count");

    t.state_count = 2;
    t.actions     = NULL;
    CHECK(!mjg_fsm_validate(&t), "missing action array");

    t.actions     = acts;
    t.transitions = NULL;
    CHECK(!mjg_fsm_validate(&t), "NULL transitions with a non-zero count");

    t.transition_count = 0;
    CHECK(mjg_fsm_validate(&t), "no transitions at all is legal");
}

static void test_bad_input_is_inert(void)
{
    fix     f;
    mjg_fsm m;

    reset(&f);

    CHECK(!mjg_fsm_init(NULL, &gated_table, 0, &f), "a NULL machine is rejected");
    CHECK(mjg_fsm_current(NULL) == MJG_FSM_NO_STATE, "a NULL machine has no state");

    CHECK(!mjg_fsm_init(&m, NULL, 0, &f), "a NULL table is rejected");
    CHECK(!mjg_fsm_init(&m, &gated_table, 99, &f), "an out-of-range initial state is rejected");
    CHECK(mjg_fsm_current(&m) == MJG_FSM_NO_STATE, "the machine is left uninitialised");

    mjg_fsm_step(&m);
    CHECK(f.entered[0] == 0, "stepping an inert machine does nothing");

    mjg_fsm_step(NULL);
    CHECK(true, "stepping a NULL machine returns instead of crashing");
}

int main(void)
{
    test_entry_and_progress();
    test_machines_are_independent();
    test_first_match_wins();
    test_null_actions_are_skipped();
    test_self_transition_repeats();
    test_one_transition_per_step();
    test_validate();
    test_bad_input_is_inert();

    if (failures == 0) {
        printf("ok    %d checks passed\n", checks);
        return 0;
    }

    printf("FAILED %d of %d checks\n", failures, checks);
    return 1;
}
