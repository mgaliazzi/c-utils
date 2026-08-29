# `kalman` — linear Kalman filter, scalar measurement

Estimates a small state vector from one noisy number at a time. It does the
thing a low-pass filter cannot: recover a quantity you never measured, by
knowing how it relates to one you did.

```c
mjg_kalman_config cfg;
mjg_kalman        kf;

/* position and velocity, with only position measured */
mjg_kalman_model_constant_velocity(&cfg, dt, 0.2f, sensor_variance);
mjg_kalman_init(&kf, &cfg, NULL, NULL);

for (;;) {
    mjg_kalman_step(&kf, 0.0f, read_position());

    float position = mjg_kalman_state(&kf, 0);
    float velocity = mjg_kalman_state(&kf, 1);   /* never measured */
}
```

## Why scalar measurements

That constraint is the whole design. With a scalar measurement the Kalman gain
needs **one division** — never a matrix inverse — however many states there
are. That is what keeps this to two files with no matrix library, no code
generation and no `math.h`.

## Using it

Copy two files into your project:

```
kalman/include/mjg_kalman.h
kalman/src/mjg_kalman.c
```

Nothing to install, nothing to link, no build step. The header needs
`<stdbool.h>`, `<stdint.h>` and `<stddef.h>`. C99.

Two build-time knobs, both optional:

| | |
|---|---|
| `MJG_KALMAN_MAX_STATES` | Largest state vector. Default 4. |
| `MJG_KALMAN_USE_DOUBLE` | Work in `double` rather than `float`. |

RAM is one `mjg_kalman` per filter — about 90 bytes at four states. The model
is separate and `const`, so it belongs in flash, and one model can back any
number of filters.

## The contract

**Predict** — `x = F·x + b·u`, `P = F·P·Fᵀ + Q`. The estimate moves the way
the model says it should, and gets less certain.

**Update** — fold in a measurement `z`:

```
y = z - H·x           how far off the model was      (scalar)
S = H·P·Hᵀ + R        how surprised it is allowed to be  (scalar)
K = P·Hᵀ / S          the gain                       (one division)
x = x + K·y
P = (I-KH)·P·(I-KH)ᵀ + K·R·Kᵀ                        (Joseph form)
```

`mjg_kalman_step()` does both, for the usual fixed-rate loop.

## Choosing Q and R

**`R` is the measurement variance** — the square of your sensor's noise
standard deviation. You can *measure* it: log a few hundred samples with the
sensor held still, and take the variance. It is not a tuning knob; it is a
property of the hardware.

**`Q` is your modelling-error budget** — how much the real system does things
your model does not describe. You cannot measure it, so it is the knob you
turn. For the constant-velocity model, `accel_noise` is the standard deviation
of the acceleration you did not model; it is what lets the filter follow
something that speeds up.

The rule that makes tuning tractable: **only the ratio matters.** Scaling `Q`
and `R` by the same factor changes nothing at all — the gain, and so the
estimate, is identical. So fix `R` at the value you measured, and tune only
`Q`.

- **Too small a `Q`** and the filter trusts its model over your sensor. It
  becomes smooth, confident and slow, and lags whenever reality departs from
  the model.
- **Too large a `Q`** and it chases every sample. In the limit it is not
  filtering at all.

Start with `Q` small, and raise it until the estimate keeps up with the
fastest real change you care about.

## Initial covariance, and a trap

`mjg_kalman_init()` takes an optional diagonal for `P`. It says *how wrong the
starting estimate might be*, and defaults to 1 per state.

If you use gating, this matters more than it looks. **An overconfident `P0`
makes the gate reject every real measurement.** The gate judges a reading
against how sure the filter thought it was, so claiming certainty you do not
have makes correct data look like outliers, and the filter never converges.
Its own `Q` will widen `P` again eventually — but at a realistic `Q` that can
take a million steps.

Set `P0` to honest ignorance. If you have no idea where you are starting, say
so with a large value; the first few measurements will collapse it.

## Outlier gating

Set `cfg.gate` to reject readings that are too far from what the model
expected. It is a threshold on normalised innovation squared, `y²/S`, so it
scales itself with the filter's own confidence rather than being an absolute
window:

| `gate` | |
|---|---|
| `0` | off, accept everything |
| `9` | three sigma — a good default |
| `16` | four sigma, more permissive |

A rejected update returns `false` and leaves `x` and `P` untouched. This costs
one comparison, needs no square root, and is the feature most small Kalman
libraries leave out despite every real sensor stream needing it.

## Control input

If you know what you commanded, tell the filter. Set `cfg.b` to how a unit of
input moves each state, and pass `u` to `mjg_kalman_predict()`.

A commanded input is known exactly, so it **moves `x` but leaves `P` alone** —
it adds no uncertainty. Leave `b` at zero (the default) and `u` is ignored.

This is what turns the filter into a proper observer. Without it, the filter
watches the plant respond to your control and has to write that motion off as
noise, so the estimate lags exactly when the controller is working hardest.
[`examples/observer.c`](examples/observer.c) runs two filters on identical
measurements, one told about `u` and one not:

```
  step  setpoint   truth   measured   informed   uninformed
    18       0.0   -0.05       0.27       0.05         0.08
  ---- setpoint step ----
    20      10.0    3.96       3.92       3.97         0.20
    24      10.0    7.72       7.49       7.75         1.30
```

### Pairing with `pid`

```c
float position = mjg_kalman_state(&kf, 0);
float u        = mjg_pid_update(&pid, setpoint, position, dt);

drive_actuator(u);
mjg_kalman_predict(&kf, u);          /* the filter knows what was commanded */
mjg_kalman_update(&kf, read_sensor());
```

`kalman` does not include or link `pid` — components here never depend on each
other. It just takes a number that `pid` happens to be a good source of.

## Several sensors

`H` and `R` are fixed per filter, so one filter reads one sensor. 

When the measurement noises are uncorrelated, folding *m* readings in **one at
a time as m scalar updates** gives a mathematically identical result to a
simultaneous vector update — and is often better conditioned than the matrix
inverse that would otherwise be needed. The only condition is that you do not
re-predict between them, which is what this API does naturally:

```c
mjg_kalman_predict(&kf, u);
mjg_kalman_update(&kf, sensor_a);
mjg_kalman_update(&kf, sensor_b);
```

Since `H` is fixed here, taking advantage of that means either one filter per
sensor, or pointing `kf.cfg` at another model between updates — both must
share the same `n`.

## What this does not do

**Vector measurements** with correlated noise. That needs a matrix inverse,
which is precisely what this component avoids. If your sensor reports a real
covariance matrix that changes at runtime — a radar range and
bearing, vision pixel errors — you want a general filter, not this one.

**Nonlinear systems.** No EKF, no UKF, no Jacobians.

## Compared to other libraries

The value here is the packaging, (maybe) the numerics
and the documentation. Credit where it belongs:

- [TinyEKF](https://github.com/simondlevy/TinyEKF) — header-only C/C++ EKF
  with static allocation and Python prototyping tools. **Use this if you need
  nonlinear.**
- [embedded-kf](https://github.com/sahil-kale/embedded-kf) — statically
  allocated linear KF in pure C, with vector measurements.
- [kalman-clib](https://github.com/sunsided/kalman-clib) — pure C, vector
  measurements, Cholesky-based inversion. Filters are
  declared through a preprocessor factory.

Where this one differs: it uses **Joseph form** and **re-symmetrises `P`**,
it has **automatic innovation gating**, and it needs **no matrix inverse and no 
tooling at all**, because it only ever handles a scalar measurement. 

## Further reading

- [Kalman filter](https://en.wikipedia.org/wiki/Kalman_filter) on Wikipedia.
- Roger Labbe,
  [*Kalman and Bayesian Filters in Python*](https://github.com/rlabbe/Kalman-and-Bayesian-Filters-in-Python)
  — free, and the clearest practical introduction there is. The chapters on
  choosing `Q` and on filter divergence are the ones to read first.
- Greg Welch and Gary Bishop, *An Introduction to the Kalman Filter* — the
  standard short technical note.

## Tests

From the repository root:

```bash
cmake -S . -B build -DCUTILS_WERROR=ON
cmake --build build
ctest --test-dir build -C Debug --output-on-failure
```
