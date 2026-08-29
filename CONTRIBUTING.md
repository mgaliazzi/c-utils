# Contributing

Thanks for looking. This repository has a few rules that are unusual enough to
be worth reading before you write any code — most of them exist to protect one
property, and a change that breaks it will be sent back however good it is
otherwise.

## The one rule that matters

**Every component must compile standalone.** No shared `common.h`, no component
including another, no build-time tooling. Copying a component's files into
an unrelated project has to just work:

```
<component>/include/mjg_<name>.h
<component>/src/mjg_<name>.c
```

That is the whole distribution model. CMake exists to build and test *this*
repository, not to ship it, which is why there are no `install()` or `export()`
rules. If a change would make someone need CMake, a package manager, a code
generator, or a second file from elsewhere in the repo, it is the wrong change.

Test it the way a user would: copy the files and a small `main.c` into an
empty directory and compile with nothing but a compiler.

```bash
cc -std=c99 -Wall -Wextra -Wpedantic -Werror -I. mjg_pid.c main.c -o t
```

## Constraints

Applies to every component:

- **C99**, set explicitly. No compiler extensions.
- **No allocation.** The caller supplies all storage. No `malloc`, no `free`,
  no VLAs, no recursion that grows with input.
- **No HAL, no vendor headers, no RTOS.** Nothing that ties a component to one
  chip or framework.
- **Minimal standard headers.** `stdbool.h`, `stdint.h`, `stddef.h` are
  expected; `float.h` is fine. Anything else needs a reason.
- **No global mutable state.** Multiple independent instances must be possible.
- **`const` correctness**, so read-only model data can live in flash.

## Naming

| | |
|---|---|
| Header | `mjg_<component>.h` |
| Functions | `mjg_<component>_<verb>()` |
| Types | `typedef struct mjg_<component> mjg_<component>;` |
| Macros | `MJG_<COMPONENT>_<NAME>` |
| File-local `static` | no prefix needed |

## Building and testing

```bash
cmake -S . -B build -DCUTILS_WERROR=ON
cmake --build build
ctest --test-dir build -C Debug --output-on-failure
```

`-C Debug` is required by multi-config generators (Visual Studio, Xcode) and
ignored by single-config ones, so that line works everywhere.

Every component can be toggled: `-DCUTILS_BUILD_PID=OFF`, and so on. CI builds
with **gcc and clang** at `-Wall -Wextra -Wpedantic -Werror`, and runs the
tests again under **ASan and UBSan**. A pull request has to be clean under all
of it.

## Tests

Tests are plain C with a `CHECK(condition, message)` macro and a failure
counter. **No test framework** for now.

## Adding a component

Same layout as the others:

```
<name>/
├── README.md               # the real documentation
├── CMakeLists.txt
├── include/mjg_<name>.h
├── src/mjg_<name>.c
├── tests/test_<name>.c
└── examples/<something>.c
```

Copy an existing `CMakeLists.txt` — they are near-identical — then add the
`option(CUTILS_BUILD_<NAME> ...)` and `add_subdirectory()` to the top-level
`CMakeLists.txt`, and a row to the component table in `README.md`.

The README is not an afterthought. For these components it is most of the
value: the algorithms are well known, and what is scarce is a clear statement
of the contract, the tuning knobs, and the failure modes. Say plainly what the
component does **not** do, and point at better tools where they exist.

Examples must be host-runnable and deterministic — use a small built-in
generator rather than `rand()`, so the printed output is identical everywhere.

## Formatting

`.clang-format` is in the repository root. Run it before opening a pull
request:

```bash
clang-format -i <files>
```

## What makes a good pull request

- One component, or one concern, per pull request.
- Tests for the behaviour you changed, and a README update if you changed
  anything the README describes.
- If you added a build-time knob, build with and without it.
- Say what you verified and on what compiler. If you only tried one, say so —
  that is useful, and much better than implying you tried all of them.
