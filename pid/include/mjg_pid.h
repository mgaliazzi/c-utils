/*
 * mjg_pid.h - PID controller with anti-windup and a filtered derivative.
 *
 * SPDX-License-Identifier: MIT
 * Copyright (c) 2021 Maicon Galiazi
 *
 * Each update takes a setpoint, a measurement and the elapsed time, and
 * returns a control output:
 *
 *     out = kp*error + ki*integral + kd*derivative
 *
 * That is the parallel ("academic") form, with three independent gains -- not the ISA
 * standard form, where one gain multiplies an integral time and a derivative
 * time. README.md gives the conversion if your gains came from tuning tables.
 *
 * The integrator freezes while the output is saturated and the error would
 * push it further in, so it cannot wind up. The derivative is low-pass
 * filtered and, by default, taken from the measurement rather than the error,
 * so a setpoint step does not kick the output. README.md has the contract.
 *
 * Needs only stdbool.h and float.h. To use it, copy this file and mjg_pid.c
 * into your project.
 */

#ifndef MJG_PID_H
#define MJG_PID_H

#include <float.h>
#include <stdbool.h>

#ifdef __cplusplus
extern "C" {
#endif

/* Define MJG_PID_USE_DOUBLE before including this header to work in double. */
#ifdef MJG_PID_USE_DOUBLE
typedef double mjg_pid_real;
#define MJG_PID_REAL_MAX DBL_MAX
#else
typedef float mjg_pid_real;
#define MJG_PID_REAL_MAX FLT_MAX
#endif

typedef struct {
    mjg_pid_real kp;
    mjg_pid_real ki; /* per second: multiplies the integral of error*dt   */
    mjg_pid_real kd; /* seconds:    multiplies the rate of change         */

    /* Low-pass weight on the derivative, in (0, 1]. 1 leaves it unfiltered;
     * smaller is smoother and slower. */
    mjg_pid_real d_filter;

    mjg_pid_real out_min;
    mjg_pid_real out_max;

    /* Bound on the stored integral, before ki is applied. */
    mjg_pid_real integral_min;
    mjg_pid_real integral_max;

    /* True takes the derivative from the measurement, so a setpoint step
     * does not kick the output. False takes it from the error. */
    bool derivative_on_measurement;
} mjg_pid_config;

/* One controller. The caller owns the storage. Treat the fields as private
 * and read the output through mjg_pid_output(). */
struct mjg_pid {
    mjg_pid_config cfg;
    mjg_pid_real   integral;
    mjg_pid_real   prev_input;      /* previous measurement, or previous error */
    mjg_pid_real   prev_derivative; /* previous filtered derivative            */
    mjg_pid_real   output;
    bool           primed;          /* false until the first update            */
};
typedef struct mjg_pid mjg_pid;

/* Fills cfg with a usable starting point: kp = 1, no integral or derivative
 * action, no filtering, no limits, derivative on measurement. */
void mjg_pid_defaults(mjg_pid_config *cfg);

/* Rejects a config that cannot work: out_min >= out_max, integral_min >
 * integral_max, or d_filter outside (0, 1]. Gains may be any value, including
 * negative for a reverse-acting loop. */
bool mjg_pid_validate(const mjg_pid_config *cfg);

/* Copies cfg into pid and clears the loop state. Returns false, leaving the
 * controller inert, if pid is NULL or the config is invalid. */
bool mjg_pid_init(mjg_pid *pid, const mjg_pid_config *cfg);

/* Advances the loop by dt seconds and returns the new output. A dt of zero or
 * less returns the previous output and changes nothing.
 *
 * Passing dt = 1 makes ki and kd per-sample rather than per-second, which is
 * what a fixed-rate loop with gains tuned by hand usually assumes. */
mjg_pid_real mjg_pid_update(mjg_pid *pid, mjg_pid_real setpoint,
                            mjg_pid_real measurement, mjg_pid_real dt);

/* Clears the integral, the derivative history and the output, keeping the
 * config. Use it whenever the loop has been off long enough that its old
 * state is meaningless. */
void mjg_pid_reset(mjg_pid *pid);

/* Seeds the integral so the loop resumes near 'output' instead of jumping
 * from zero, for handing over from manual control. Needs a non-zero ki to
 * have any effect on later updates. */
void mjg_pid_preload(mjg_pid *pid, mjg_pid_real output);

/* The most recent output, or 0 if the controller is uninitialised. */
mjg_pid_real mjg_pid_output(const mjg_pid *pid);

#ifdef __cplusplus
} /* extern "C" */
#endif

#endif /* MJG_PID_H */
