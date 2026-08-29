# c-utils

Small, independent C components for embedded work. Each one solves a single
problem, has no dependencies, and is meant to be **copied into your project**
rather than installed.

## Components

| | |
|---|---|
| [`fsm`](fsm/) | Table-driven state machine with polled conditions. No event queue, no allocation. |
| [`pid`](pid/) | PID controller with anti-windup, a filtered derivative and output limits. |
| [`kalman`](kalman/) | Linear Kalman filter for a scalar measurement. Joseph form, outlier gating, no matrix inverse. |

## The independence rule

Every component compiles standalone. There is no shared `common.h`, and no
component includes another. Copying a component's two files into an unrelated
project must just work — that is the property everything here is designed
around, and it is what keeps the components cheap to adopt and cheap to split
out again later.

## Using a component

Copy its header and source into your tree:

```
fsm/include/mjg_fsm.h
fsm/src/mjg_fsm.c
```

That is the supported integration path. There is no package to install and no
registry entry to chase; each file carries its own licence header so it stays
self-describing once it leaves this repository.

## Conventions

- Public symbols and header filenames are prefixed `mjg_` — `mjg_fsm.h`,
  `mjg_fsm_init()`, `typedef struct mjg_fsm mjg_fsm;`
- Macros and constants are `MJG_FSM_*`
- File-local `static` functions carry no prefix
- No `malloc` in any core API — the caller supplies storage
- C99, set explicitly, for reach across older embedded toolchains

## Building

CMake exists to build and test *this* repository, not to distribute it. Each
component can be toggled independently:

```bash
cmake -S . -B build -DCUTILS_WERROR=ON
cmake --build build
ctest --test-dir build -C Debug --output-on-failure
```

`-C Debug` is required by multi-config generators (Visual Studio, Xcode) and
ignored by single-config ones, so the same line works everywhere.

| Option | Default | |
|---|---|---|
| `CUTILS_BUILD_FSM` | `ON` | Build the state machine component |
| `CUTILS_BUILD_TESTS` | `ON` | Build and register component tests |
| `CUTILS_BUILD_EXAMPLES` | `ON` | Build component examples |
| `CUTILS_WERROR` | `OFF` | Treat warnings as errors (CI turns this on) |

Components export a namespaced alias — `c-utils::fsm` — for in-tree and
`FetchContent` use. There are deliberately no `install()` or `export()` rules.


## Licence

MIT. See [LICENSE](LICENSE).
