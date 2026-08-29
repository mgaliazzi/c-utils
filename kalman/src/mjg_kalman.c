/*
 * mjg_kalman.c - linear Kalman filter for a scalar measurement.
 *
 * SPDX-License-Identifier: MIT
 * Copyright (c) 2026 Maicon Galiazi
 *
 * See mjg_kalman.h for the contract.
 */

#include "mjg_kalman.h"

#define N_MAX MJG_KALMAN_MAX_STATES

static void clear_config(mjg_kalman_config *cfg)
{
    uint8_t i, j;

    for (i = 0u; i < N_MAX; ++i) {
        cfg->b[i] = (mjg_kalman_real)0;
        cfg->h[i] = (mjg_kalman_real)0;
        for (j = 0u; j < N_MAX; ++j) {
            cfg->f[i][j] = (mjg_kalman_real)0;
            cfg->q[i][j] = (mjg_kalman_real)0;
        }
    }
}

void mjg_kalman_model_scalar(mjg_kalman_config *cfg,
                             mjg_kalman_real    process_var,
                             mjg_kalman_real    meas_var)
{
    if (cfg == NULL) {
        return;
    }

    clear_config(cfg);

    cfg->n       = 1u;
    cfg->f[0][0] = (mjg_kalman_real)1;
    cfg->q[0][0] = process_var;
    cfg->h[0]    = (mjg_kalman_real)1;
    cfg->r       = meas_var;
    cfg->gate    = (mjg_kalman_real)0;
}

void mjg_kalman_model_constant_velocity(mjg_kalman_config *cfg,
                                        mjg_kalman_real    dt,
                                        mjg_kalman_real    accel_noise,
                                        mjg_kalman_real    meas_var)
{
    mjg_kalman_real var, dt2, dt3, dt4;

    if (cfg == NULL) {
        return;
    }

    clear_config(cfg);

    var = accel_noise * accel_noise;
    dt2 = dt * dt;
    dt3 = dt2 * dt;
    dt4 = dt2 * dt2;

    cfg->n = 2u;

    /* position advances by velocity * dt */
    cfg->f[0][0] = (mjg_kalman_real)1;
    cfg->f[0][1] = dt;
    cfg->f[1][0] = (mjg_kalman_real)0;
    cfg->f[1][1] = (mjg_kalman_real)1;

    /* Discrete white-noise acceleration: an unknown acceleration acting for
     * one step moves position by a*dt*dt/2 and velocity by a*dt. */
    cfg->q[0][0] = var * dt4 / (mjg_kalman_real)4;
    cfg->q[0][1] = var * dt3 / (mjg_kalman_real)2;
    cfg->q[1][0] = var * dt3 / (mjg_kalman_real)2;
    cfg->q[1][1] = var * dt2;

    /* position is measured, velocity is inferred from how it changes */
    cfg->h[0] = (mjg_kalman_real)1;
    cfg->h[1] = (mjg_kalman_real)0;

    cfg->r    = meas_var;
    cfg->gate = (mjg_kalman_real)0;
}

bool mjg_kalman_validate(const mjg_kalman_config *cfg)
{
    if (cfg == NULL) {
        return false;
    }
    if (cfg->n == 0u || cfg->n > N_MAX) {
        return false;
    }
    if (cfg->r <= (mjg_kalman_real)0) {
        return false;
    }
    if (cfg->gate < (mjg_kalman_real)0) {
        return false;
    }

    return true;
}

bool mjg_kalman_init(mjg_kalman              *kf,
                     const mjg_kalman_config *cfg,
                     const mjg_kalman_real   *x0,
                     const mjg_kalman_real   *p0_diag)
{
    uint8_t i, j;

    if (kf == NULL) {
        return false;
    }

    /* Stay inert on failure, so an ignored return value shows up as nothing
     * happening rather than as a filter running on a garbage model. */
    kf->cfg            = NULL;
    kf->innovation     = (mjg_kalman_real)0;
    kf->innovation_var = (mjg_kalman_real)0;
    kf->accepted       = false;

    for (i = 0u; i < N_MAX; ++i) {
        kf->x[i] = (mjg_kalman_real)0;
        for (j = 0u; j < N_MAX; ++j) {
            kf->p[i][j] = (mjg_kalman_real)0;
        }
    }

    if (!mjg_kalman_validate(cfg)) {
        return false;
    }

    for (i = 0u; i < cfg->n; ++i) {
        kf->x[i]    = (x0 != NULL) ? x0[i] : (mjg_kalman_real)0;
        kf->p[i][i] = (p0_diag != NULL) ? p0_diag[i] : (mjg_kalman_real)1;
    }

    kf->cfg = cfg;

    return true;
}

void mjg_kalman_predict(mjg_kalman *kf, mjg_kalman_real u)
{
    const mjg_kalman_config *cfg;
    mjg_kalman_real          xn[N_MAX];
    mjg_kalman_real          fp[N_MAX][N_MAX]; /* F*P */
    mjg_kalman_real          sum;
    uint8_t                  n, i, j, k;

    if (kf == NULL || kf->cfg == NULL) {
        return;
    }
    cfg = kf->cfg;
    n   = cfg->n;

    /* x = F*x + b*u. A commanded input is known exactly, so it never appears
     * in the covariance below. */
    for (i = 0u; i < n; ++i) {
        sum = cfg->b[i] * u;
        for (j = 0u; j < n; ++j) {
            sum += cfg->f[i][j] * kf->x[j];
        }
        xn[i] = sum;
    }
    for (i = 0u; i < n; ++i) {
        kf->x[i] = xn[i];
    }

    /* P = F*P*Ft + Q, in two passes through one temporary. */
    for (i = 0u; i < n; ++i) {
        for (j = 0u; j < n; ++j) {
            sum = (mjg_kalman_real)0;
            for (k = 0u; k < n; ++k) {
                sum += cfg->f[i][k] * kf->p[k][j];
            }
            fp[i][j] = sum;
        }
    }
    for (i = 0u; i < n; ++i) {
        for (j = 0u; j < n; ++j) {
            sum = cfg->q[i][j];
            for (k = 0u; k < n; ++k) {
                sum += fp[i][k] * cfg->f[j][k];
            }
            kf->p[i][j] = sum;
        }
    }
}

bool mjg_kalman_update(mjg_kalman *kf, mjg_kalman_real z)
{
    const mjg_kalman_config *cfg;
    mjg_kalman_real          ph[N_MAX];        /* P*Ht          */
    mjg_kalman_real          gain[N_MAX];      /* K             */
    mjg_kalman_real          w[N_MAX];         /* (A*P)*Ht      */
    mjg_kalman_real          ap[N_MAX][N_MAX]; /* A*P, A = I-KH */
    mjg_kalman_real          hx, y, s, sum, mean;
    uint8_t                  n, i, j;

    if (kf == NULL || kf->cfg == NULL) {
        return false;
    }
    cfg = kf->cfg;
    n   = cfg->n;

    /* Innovation: how far the measurement is from what the model expected. */
    hx = (mjg_kalman_real)0;
    for (i = 0u; i < n; ++i) {
        hx += cfg->h[i] * kf->x[i];
    }
    y = z - hx;

    for (i = 0u; i < n; ++i) {
        sum = (mjg_kalman_real)0;
        for (j = 0u; j < n; ++j) {
            sum += kf->p[i][j] * cfg->h[j];
        }
        ph[i] = sum;
    }

    s = cfg->r;
    for (i = 0u; i < n; ++i) {
        s += cfg->h[i] * ph[i];
    }

    kf->innovation     = y;
    kf->innovation_var = s;

    if (s <= (mjg_kalman_real)0) { /* unreachable while r > 0 and P is sane */
        kf->accepted = false;
        return false;
    }

    /* Outlier gate on normalised innovation squared. Needs no square root. */
    if (cfg->gate > (mjg_kalman_real)0 && (y * y) > (cfg->gate * s)) {
        kf->accepted = false;
        return false;
    }

    /* One division, whatever n is. This is the whole reason the component
     * needs no matrix inverse. */
    for (i = 0u; i < n; ++i) {
        gain[i] = ph[i] / s;
    }

    for (i = 0u; i < n; ++i) {
        kf->x[i] += gain[i] * y;
    }

    /*
     * Joseph form, P = (I-KH)*P*(I-KH)t + K*R*Kt, written out rather than by
     * building (I-KH). With A = I-KH and P symmetric, (A*P)[i][j] collapses to
     * P[i][j] - K[i]*ph[j], and the second product to ap[i][j] - K[j]*w[i].
     * That keeps the whole update to one temporary matrix.
     *
     * The short form P = (I-KH)*P is cheaper, but it loses symmetry and then
     * positive-definiteness under rounding, and the filter degrades quietly.
     */
    for (i = 0u; i < n; ++i) {
        for (j = 0u; j < n; ++j) {
            ap[i][j] = kf->p[i][j] - (gain[i] * ph[j]);
        }
    }
    for (i = 0u; i < n; ++i) {
        sum = (mjg_kalman_real)0;
        for (j = 0u; j < n; ++j) {
            sum += ap[i][j] * cfg->h[j];
        }
        w[i] = sum;
    }
    for (i = 0u; i < n; ++i) {
        for (j = 0u; j < n; ++j) {
            kf->p[i][j] = ap[i][j] - (gain[j] * w[i]) + (cfg->r * gain[i] * gain[j]);
        }
    }

    /* Symmetric in exact arithmetic, not in floating point. Fold it back
     * before the asymmetry has anywhere to accumulate. */
    for (i = 0u; i < n; ++i) {
        for (j = (uint8_t)(i + 1u); j < n; ++j) {
            mean        = (kf->p[i][j] + kf->p[j][i]) / (mjg_kalman_real)2;
            kf->p[i][j] = mean;
            kf->p[j][i] = mean;
        }
    }

    kf->accepted = true;

    return true;
}

bool mjg_kalman_step(mjg_kalman *kf, mjg_kalman_real u, mjg_kalman_real z)
{
    mjg_kalman_predict(kf, u);
    return mjg_kalman_update(kf, z);
}

mjg_kalman_real mjg_kalman_state(const mjg_kalman *kf, uint8_t i)
{
    if (kf == NULL || kf->cfg == NULL || i >= kf->cfg->n) {
        return (mjg_kalman_real)0;
    }
    return kf->x[i];
}

mjg_kalman_real mjg_kalman_covariance(const mjg_kalman *kf, uint8_t i, uint8_t j)
{
    if (kf == NULL || kf->cfg == NULL || i >= kf->cfg->n || j >= kf->cfg->n) {
        return (mjg_kalman_real)0;
    }
    return kf->p[i][j];
}

mjg_kalman_real mjg_kalman_innovation(const mjg_kalman *kf)
{
    if (kf == NULL) {
        return (mjg_kalman_real)0;
    }
    return kf->innovation;
}
