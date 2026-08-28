# `fsm` — table-driven state machine

A state machine you declare as a table and drive from a loop. No event type,
no queue, no dispatcher, no allocation, no clock of its own.

```c
static const mjg_fsm_transition transitions[] = {
    { ST_IDLE,    start_requested, ST_RUNNING },
    { ST_RUNNING, fault_detected,  ST_FAULT   },
    { ST_RUNNING, stop_requested,  ST_IDLE    },
    { ST_FAULT,   fault_cleared,   ST_IDLE    },
};
```

That table *is* the state diagram. Reading it tells you the whole machine.

## When this fits

Use it when your control flow is already a loop that polls things — a
super-loop, a periodic timer callback, a cyclic task. Once per pass you ask
the machine "may I move yet?", and it asks your condition functions.

| | |
|---|---|
| **Polled conditions** (this) | The answers are already variables you can read: a flag, an elapsed time, a sensor value. Nothing to plumb. |
| **Event queue** | Better when events are truly asynchronous and must not be missed between polls — an interrupt that fires once and is gone. |
| **Hierarchical statechart** | Better when you need nested states, exit actions, history, or parallel regions. Much more machinery. |

If you have been writing `switch` pyramids that slowly rot, this is the shape
you were reaching for.

## Using it

Copy two files into your project:

```
fsm/include/mjg_fsm.h
fsm/src/mjg_fsm.c
```

That is the whole integration story. Nothing to install, nothing to link. The
header needs `<stdbool.h>`, `<stdint.h>` and `<stddef.h>` and nothing else —
no HAL, no `malloc`, no `stdio`, no time source. C99.

### A complete machine

```c
#include "mjg_fsm.h"

enum { ST_IDLE, ST_RUNNING, ST_FAULT };

typedef struct { bool start, stop, fault; } io;

static void enter_idle(void *u)    { (void)u; motor_off(); }
static void enter_running(void *u) { (void)u; motor_on();  }
static void enter_fault(void *u)   { (void)u; motor_off(); latch_alarm(); }

static bool start_requested(void *u) { return ((io *)u)->start; }
static bool stop_requested(void *u)  { return ((io *)u)->stop;  }
static bool fault_detected(void *u)  { return ((io *)u)->fault; }
static bool fault_cleared(void *u)   { return !((io *)u)->fault; }

static const mjg_fsm_action_fn actions[] = {
    enter_idle, enter_running, enter_fault
};

static const mjg_fsm_transition transitions[] = {
    { ST_IDLE,    start_requested, ST_RUNNING },
    { ST_RUNNING, fault_detected,  ST_FAULT   },   /* checked before stop */
    { ST_RUNNING, stop_requested,  ST_IDLE    },
    { ST_FAULT,   fault_cleared,   ST_IDLE    },
};

static const mjg_fsm_table table = MJG_FSM_TABLE(actions, transitions);

int main(void)
{
    io      pins = { false, false, false };
    mjg_fsm machine;

    if (!mjg_fsm_init(&machine, &table, ST_IDLE, &pins)) {
        return 1;   /* the table is malformed */
    }

    for (;;) {
        read_inputs(&pins);
        mjg_fsm_step(&machine);
    }
}
```

`MJG_FSM_TABLE` derives both counts from the arrays, so a table can never
disagree with the arrays it describes.

## The contract

Each `mjg_fsm_step()` does exactly this:

1. If an entry action is pending, run it.
2. Walk the transition array **in order** for the first row that leaves the
   current state and whose condition is true.
3. If one is found, enter its `to` state and run that state's entry action
   **immediately, in the same step**.

Four consequences are worth knowing by heart.

**Array order is priority order.** When two rows leave the same state and both
conditions are true, the earlier one wins. Put error and abort checks above
normal progress — that ordering is the only thing making them pre-emptive.

**An action is an *entry* action, and it runs once per entry.** Hold a state
for a thousand steps and the action ran once. Put one-shot work there (start a
timer, energise an output) and continuous work in the loop around the machine.

**A self-transition turns an entry action into a repeated action.** A row
`{ S, condition, S }` re-enters `S`, so its action runs again every step the
condition holds:

```c
{ ST_POLLING, NULL, ST_POLLING },   /* action runs every single step */
```

That is a useful idiom for "keep doing this until something else becomes
true", but it is easy to write by accident, so it is worth recognising.

**At most one transition per step.** A chain of unconditional rows advances one
state per step rather than running to completion inside one call. This is what
stops a self-transition from spinning forever inside `mjg_fsm_step()`.

## Conventions

**A `NULL` condition means "always true"** — an unconditional transition. It
saves writing the `return true;` stub these tables otherwise fill up with:

```c
{ ST_INIT, NULL, ST_IDLE },   /* falls straight through on the next step */
```

**A `NULL` action means the state has no entry action.** The array still needs
an entry per state; just put `NULL` in it.

## Validation

`mjg_fsm_validate()` catches the mistakes that would otherwise become a wild
jump at run time: an out-of-range `from` or `to`, a missing action array, a
zero or oversized `state_count`, a NULL transition array with a non-zero count.

`mjg_fsm_init()` calls it and refuses a bad table, leaving the machine inert —
stepping it does nothing rather than calling through a garbage pointer. Call
`mjg_fsm_validate()` directly only to check a table on its own.

## Several machines

The table is `const` and stateless; all the changing state lives in `mjg_fsm`.
One table can back any number of machines, each with its own `user` pointer:

```c
mjg_fsm axis_x, axis_y;
mjg_fsm_init(&axis_x, &table, ST_IDLE, &x_context);
mjg_fsm_init(&axis_y, &table, ST_IDLE, &y_context);
```

## Cost

RAM is one `mjg_fsm` per machine — a pointer, a `void *`, a `uint8_t` and a
`bool`. The tables are `const` and belong in flash.

A step is O(`transition_count`): the scan is linear over the whole array. At
the sizes this style of machine reaches — a few dozen rows, stepped at tens or
hundreds of hertz — that is not worth optimising. If it ever is, sorting the
table by `from` and storing a per-state start offset makes it O(rows leaving
*this* state). That is deliberately not built: it would trade the table's
readability, which is the main thing this component is for, against a cost
nobody has paid yet.

## API

| | |
|---|---|
| `mjg_fsm_init(fsm, table, initial, user)` | Validates and initialises. Returns `false` and leaves the machine inert on a bad table or state. |
| `mjg_fsm_step(fsm)` | Advances one step, per the contract above. |
| `mjg_fsm_current(fsm)` | Current state id, or `MJG_FSM_NO_STATE` if uninitialised. |
| `mjg_fsm_validate(table)` | Checks a table without instantiating it. |
| `MJG_FSM_TABLE(actions, transitions)` | Table initialiser with both counts derived from the arrays. |
| `MJG_FSM_COUNT_OF(arr)` | Element count of a fixed-size array. |
| `MJG_FSM_MAX_STATES` | Largest allowed `state_count` (255), so the largest valid id is 254. |
| `MJG_FSM_NO_STATE` | Sentinel for an uninitialised machine. |

## Tests

From the repository root:

```bash
cmake -S . -B build -DCUTILS_WERROR=ON
cmake --build build
ctest --test-dir build -C Debug --output-on-failure
```

`-C Debug` is required by multi-config generators (Visual Studio, Xcode) and
ignored by single-config ones, so the same line works everywhere.

The worked example in [`examples/traffic_light.c`](examples/traffic_light.c)
builds alongside the tests — a crossing where a button press pre-empts the
green timeout, which is the priority rule made visible. Run it from
`build/fsm/fsm_traffic_light`, or `build/fsm/Debug/fsm_traffic_light.exe` with
a multi-config generator.
