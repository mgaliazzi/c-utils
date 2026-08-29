# `pid` — PID controller

Holds a measured value at a setpoint. Anti-windup, a filtered derivative, and
output limits, in about 100 lines. No allocation, no HAL, no clock of its own.

```c
mjg_pid_config cfg;
mjg_pid        pid;

mjg_pid_defaults(&cfg);
cfg.kp      = 5.0f;
cfg.ki      = 0.5f;
cfg.kd      = 2.0f;
cfg.out_min = 0.0f;
cfg.out_max = 100.0f;

mjg_pid_init(&pid, &cfg);

for (;;) {
    float power = mjg_pid_update(&pid, setpoint, read_temperature(), dt);
    set_heater(power);
}
```

## Using it

Copy two files into your project:

```
pid/include/mjg_pid.h
pid/src/mjg_pid.c
```

Nothing to install, nothing to link. The header needs `<stdbool.h>` and
`<float.h>` — both freestanding headers, so this works on bare metal. No
`math.h`, no `malloc`, no time source. C99.

Work in `double` instead of `float` by defining `MJG_PID_USE_DOUBLE` for the
whole project. `mjg_pid_real` follows, and `MJG_PID_REAL_MAX` with it.

## The contract

Each `mjg_pid_update(pid, setpoint, measurement, dt)`:

1. A `dt` of zero or less returns the previous output and changes nothing.
2. `error = setpoint - measurement`
3. Take the rate of change — zero on the very first update, so a cold start
   cannot spike — scale it by `kd`, and blend it with the previous filtered
   value.
4. Add `error * dt` to the integral, **unless** the previous output was
   already saturated and this error would push it further in. Clamp to the
   integral limits.
5. `out = kp*error + ki*integral + derivative`, clamped to the output limits.

## PID structure

This is the **parallel form**, also called academic, non-interacting or independent
gains:

```
u(t) = Kp*e(t) + Ki*∫e(t)dt + Kd*de(t)/dt
```

The three gains are independent: changing `kp` does not rescale the integral
or derivative action.

The other one you will meet is the **standard form**, usually called the ISA
or industrial form. A single controller gain multiplies everything, and the
other two terms are expressed as *times* in seconds:

```
u(t) = Kc*[ e(t) + (1/Ti)*∫e(t)dt + Td*de(t)/dt ]
```

Tuning tables, PLC function blocks and most process-control literature are
written in the standard form. Converting into this controller:

| Standard form | Here |
|---|---|
| controller gain `Kc` | `kp = Kc` |
| integral time `Ti` (seconds) | `ki = Kc / Ti` |
| derivative time `Td` (seconds) | `kd = Kc * Td` |


A `ki` of zero gives an infinite `Ti`, which simply means no integral action.
But converting back needs a non-zero `kp`: with `kp = 0` there is no `Kc` to
factor out, and the standard form cannot express the controller at all. The
parallel form has no such gap, which is one reason it suits code better.

## What `dt` means for your gains

`ki` is per second and `kd` is in seconds, so the gains keep their meaning
when the loop rate changes or jitters.

If you have gains that were tuned by hand on a fixed-rate loop, they are
almost certainly *per sample* — the tuning absorbed the rate. **Pass
`dt = 1.0` and they carry over unchanged.**

## Anti-windup

When the output is pinned at a limit, extra error still accumulates in the
integral if you let it. The loop then has to unwind all of that before the
output leaves the limit, which shows up as a long overshoot.

This controller uses conditional integration: while the output sits at a
limit and the error would drive it further in, the integral simply does not
accumulate. It resumes the instant the error reverses, so recovery is
immediate rather than delayed.

The saturation test looks at the *previous* output, because this update's
output does not exist yet. That is the standard formulation.

There are two separate clamps and they do different jobs. `out_min`/`out_max`
bound what you actually command. `integral_min`/`integral_max` bound the
stored integral before `ki` is applied, which caps how far the loop can drift
during a long disturbance. Leave the integral limits wide unless you have a
reason not to.

## Derivative kick

By default the derivative comes from the **measurement**, not the error. A
setpoint step then produces no derivative term at all, because the
measurement has not moved yet.

Take it from the error instead — `derivative_on_measurement = false` — and the
same step produces a large, brief spike. That is *derivative kick*, and it is
almost never what you want. The option exists because some hand-tuned loops
were tuned with it and depend on it.

[`examples/heater.c`](examples/heater.c) runs both side by side on the same
numbers. The columns agree everywhere except the setpoint step:

```
   47.5s   60.0   60.07   39.9%   39.9%
  ---- setpoint step ----
   50.0s   65.0   60.17   65.1%   85.1%   <- on-error kicks
   52.5s   65.0   62.44   56.5%   56.6%
```

## Derivative filter

Raw derivatives amplify measurement noise badly. `d_filter` is a low-pass
weight in `(0, 1]`:

- `1` leaves the derivative unfiltered.
- Smaller is smoother and slower. `0.2` means each update moves one fifth of
  the way to the new value.

The weight is per update, not per second, so it is worth revisiting if you
change the loop rate a lot.

## Starting from a known output

`mjg_pid_reset()` clears the integral, the derivative history and the output.
Use it whenever the loop has been off long enough that its state is stale.

`mjg_pid_preload(pid, output)` seeds the integral so the loop resumes near
`output` instead of jumping from zero — for handing over from manual control.
It needs a non-zero `ki` to have any effect.

## Validation

`mjg_pid_validate()` rejects a config that cannot work: an empty or inverted
output range, an inverted integral range, or a `d_filter` outside `(0, 1]`.
`mjg_pid_init()` calls it and refuses a bad config.

Gains may be any value. A negative `kp` is legal and is how you run a
reverse-acting loop, where more output means less measurement.

## Several loops

Nothing is global. One config can initialise any number of controllers, each
with its own state:

```c
mjg_pid zone_a, zone_b;
mjg_pid_init(&zone_a, &cfg);
mjg_pid_init(&zone_b, &cfg);
```

## API

| | |
|---|---|
| `mjg_pid_defaults(cfg)` | Fills a config with a usable starting point. |
| `mjg_pid_validate(cfg)` | Checks a config without instantiating it. |
| `mjg_pid_init(pid, cfg)` | Copies the config in and clears the state. `false` on a bad config. |
| `mjg_pid_update(pid, setpoint, measurement, dt)` | Advances the loop, returns the new output. |
| `mjg_pid_reset(pid)` | Clears integral, derivative history and output. |
| `mjg_pid_preload(pid, output)` | Seeds the integral to resume near `output`. |
| `mjg_pid_output(pid)` | The most recent output. |
| `MJG_PID_REAL_MAX` | Largest `mjg_pid_real`, for "no limit". |

## Tests

From the repository root:

```bash
cmake -S . -B build -DCUTILS_WERROR=ON
cmake --build build
ctest --test-dir build -C Debug --output-on-failure
```

`-C Debug` is required by multi-config generators (Visual Studio, Xcode) and
ignored by single-config ones, so the same line works everywhere.

The worked example in [`examples/heater.c`](examples/heater.c) builds
alongside the tests. Run it from `build/pid/pid_heater`, or
`build/pid/Debug/pid_heater.exe` with a multi-config generator.

## Further reading and credits

This component implements a well-known algorithm. The ideas belong to the
sources below.

- [PID controller](https://en.wikipedia.org/wiki/PID_controller) on Wikipedia
  — the parallel and standard forms, integral windup, and setpoint-step
  handling, all in one place.
- Brett Beauregard, [*Improving the Beginner's PID*](http://brettbeauregard.com/blog/2011/04/improving-the-beginners-pid-introduction/)
  — a short, practical series on precisely the problems this component
  addresses, including
  [derivative kick](http://brettbeauregard.com/blog/2011/04/improving-the-beginners-pid-derivative-kick/),
  windup and bumpless transfer. Three of the design choices here follow its
  reasoning directly. Companion code:
  [br3ttb/Arduino-PID-Library](https://github.com/br3ttb/Arduino-PID-Library).
  (That blog is http-only.)
- Karl Johan Åström and Richard M. Murray,
  [*Feedback Systems: An Introduction for Scientists and Engineers*](https://fbswiki.org/wiki/index.php/Main_Page)
  — free PDF. The PID chapter treats the forms, windup and derivative
  filtering properly.
- Karl Johan Åström and Tore Hägglund, *PID Controllers: Theory, Design, and
  Tuning*, 2nd ed., ISA, 1995 — the standard reference for depth.
