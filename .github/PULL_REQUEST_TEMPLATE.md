# What this changes

<!-- One or two sentences. If it fixes an issue, link it. -->

## Why

<!-- What was wrong, or what was missing. -->

## Checklist

- [ ] Builds clean at `-Wall -Wextra -Wpedantic -Werror` (`-DCUTILS_WERROR=ON`)
- [ ] `ctest --test-dir build -C Debug --output-on-failure` passes
- [ ] Tests cover the behaviour I changed
- [ ] README updated, if I changed anything it describes
- [ ] `clang-format -i` run on the files I touched

**The independence rule** — a component must still work as copied files:

- [ ] No new dependency on another component, and no shared header
- [ ] No new required build tooling, code generation or package manager
- [ ] Still compiles standalone:
      `cc -std=c99 -Wall -Wextra -Wpedantic -Werror -I. mjg_<name>.c main.c`

## What I verified, and where

<!--
Compiler and platform, please. "gcc 13 on Linux only" is a genuinely useful
answer - much better than leaving it blank and implying everything was tried.
CI will cover gcc, clang and the sanitizers.
-->
