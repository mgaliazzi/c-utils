/*
 * mjg_kalman.h - linear Kalman filter for a scalar measurement.
 *
 * SPDX-License-Identifier: MIT
 * Copyright (c) 2026 Maicon Galiazi
 *
 * Estimates a small state vector from one noisy number at a time:
 *
 *     predict:  x = F*x + b*u        P = F*P*F' + Q
 *     update:   x += K*(z - H*x)     P = Joseph form
 *
 * Because the measurement is a scalar, the gain needs one division and never
 * a matrix inverse, however many states there are. That is what keeps this
 * to two files with no matrix library and no math.h. README.md has the
 * contract, how to pick Q and R, and what is out of scope.
 *
 * Needs only stdbool.h, stdint.h and stddef.h. To use it, copy this file and
 * mjg_kalman.c into your project.
 */

#ifndef MJG_KALMAN_H
#define MJG_KALMAN_H

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/* Largest state vector. Override at build time if you need more. */
#ifndef MJG_KALMAN_MAX_STATES
#define MJG_KALMAN_MAX_STATES 4
#endif

/* Define MJG_KALMAN_USE_DOUBLE before including this header to work in double. */
#ifdef MJG_KALMAN_USE_DOUBLE
typedef double mjg_kalman_real;
#else
typedef float mjg_kalman_real;
#endif

/*
 * The model. Nothing in here changes as the filter runs, so declare it const
 * and let it live in flash. One config can back any number of filters.
 */
typedef struct {
    uint8_t n; /* number of states, 1 .. MJG_KALMAN_MAX_STATES */

    /* State transition: x = F*x */
    mjg_kalman_real f[MJG_KALMAN_MAX_STATES][MJG_KALMAN_MAX_STATES];

    /* Process noise covariance: how much the model is trusted. */
    mjg_kalman_real q[MJG_KALMAN_MAX_STATES][MJG_KALMAN_MAX_STATES];

    /* Control column: how a commanded input moves each state. Zero to ignore. */
    mjg_kalman_real b[MJG_KALMAN_MAX_STATES];

    /* Measurement row: which states the sensor sees, and how. */
    mjg_kalman_real h[MJG_KALMAN_MAX_STATES];

    /* Measurement noise variance. Must be positive. */
    mjg_kalman_real r;

    /* Outlier gate on normalised innovation squared. An update is rejected
     * when (z - H*x)^2 / S exceeds this. Zero disables gating. */
    mjg_kalman_real gate;
} mjg_kalman_config;

/* One filter. The caller owns the storage. Treat the fields as private and
 * read through the accessors. */
struct mjg_kalman {
    const mjg_kalman_config *cfg;
    mjg_kalman_real          x[MJG_KALMAN_MAX_STATES];
    mjg_kalman_real          p[MJG_KALMAN_MAX_STATES][MJG_KALMAN_MAX_STATES];
    mjg_kalman_real          innovation;
    mjg_kalman_real          innovation_var;
    bool                     accepted; /* did the last update pass the gate */
};
typedef struct mjg_kalman mjg_kalman;

/*
 * Builds a one-state smoother: a single value, measured directly.
 * process_var says how much the value is expected to move between updates.
 */
void mjg_kalman_model_scalar(mjg_kalman_config *cfg,
                             mjg_kalman_real    process_var,
                             mjg_kalman_real    meas_var);

/*
 * Builds a two-state constant-velocity model - position and velocity, with
 * only position measured. accel_noise is the standard deviation of the
 * acceleration the model does not know about; it is what lets the filter
 * follow something that speeds up or slows down.
 */
void mjg_kalman_model_constant_velocity(mjg_kalman_config *cfg,
                                        mjg_kalman_real    dt,
                                        mjg_kalman_real    accel_noise,
                                        mjg_kalman_real    meas_var);

/* Rejects a config that cannot work: n out of range, a non-positive r, a
 * negative gate, or NULL. */
bool mjg_kalman_validate(const mjg_kalman_config *cfg);

/*
 * Points the filter at a model and sets the starting estimate. x0 may be NULL
 * for an all-zero state; p0_diag may be NULL for an identity covariance.
 * Returns false, leaving the filter inert, on a NULL filter or bad config.
 */
bool mjg_kalman_init(mjg_kalman              *kf,
                     const mjg_kalman_config *cfg,
                     const mjg_kalman_real   *x0,
                     const mjg_kalman_real   *p0_diag);

/*
 * Advances the estimate one step. 'u' is the control input commanded since
 * the last predict; pass 0 when there is none. A known input carries no
 * uncertainty, so it moves x but leaves P alone.
 */
void mjg_kalman_predict(mjg_kalman *kf, mjg_kalman_real u);

/*
 * Folds in one measurement. Returns false if the gate rejected it, in which
 * case x and P are untouched.
 */
bool mjg_kalman_update(mjg_kalman *kf, mjg_kalman_real z);

/* predict then update, for the usual fixed-rate loop. */
bool mjg_kalman_step(mjg_kalman *kf, mjg_kalman_real u, mjg_kalman_real z);

/* Estimate of state i, or 0 if i is out of range or the filter is inert. */
mjg_kalman_real mjg_kalman_state(const mjg_kalman *kf, uint8_t i);

/* Covariance element (i,j). The diagonal is the variance of each state, so
 * it is the filter's own opinion of how sure it is. */
mjg_kalman_real mjg_kalman_covariance(const mjg_kalman *kf, uint8_t i, uint8_t j);

/* The last innovation, z - H*x. Persistently large means the model is wrong. */
mjg_kalman_real mjg_kalman_innovation(const mjg_kalman *kf);

#ifdef __cplusplus
} /* extern "C" */
#endif

#endif /* MJG_KALMAN_H */
