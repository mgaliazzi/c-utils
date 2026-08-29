/*
 * mjg_pid.c - PID controller with anti-windup and a filtered derivative.
 *
 * SPDX-License-Identifier: MIT
 * Copyright (c) 2021 Maicon Galiazi
 *
 * See mjg_pid.h for the contract.
 */

#include <stddef.h>

#include "mjg_pid.h"

static mjg_pid_real clamp(mjg_pid_real value, mjg_pid_real lo, mjg_pid_real hi)
{
    if (value < lo) {
        return lo;
    }
    if (value > hi) {
        return hi;
    }
    return value;
}

void mjg_pid_defaults(mjg_pid_config *cfg)
{
    if (cfg == NULL) {
        return;
    }

    cfg->kp                       = (mjg_pid_real)1;
    cfg->ki                       = (mjg_pid_real)0;
    cfg->kd                       = (mjg_pid_real)0;
    cfg->d_filter                 = (mjg_pid_real)1;
    cfg->out_min                  = -MJG_PID_REAL_MAX;
    cfg->out_max                  = MJG_PID_REAL_MAX;
    cfg->integral_min             = -MJG_PID_REAL_MAX;
    cfg->integral_max             = MJG_PID_REAL_MAX;
    cfg->derivative_on_measurement = true;
}

bool mjg_pid_validate(const mjg_pid_config *cfg)
{
    if (cfg == NULL) {
        return false;
    }
    if (cfg->out_min >= cfg->out_max) {
        return false;
    }
    if (cfg->integral_min > cfg->integral_max) {
        return false;
    }
    if (cfg->d_filter <= (mjg_pid_real)0 || cfg->d_filter > (mjg_pid_real)1) {
        return false;
    }

    return true;
}

void mjg_pid_reset(mjg_pid *pid)
{
    if (pid == NULL) {
        return;
    }

    pid->integral        = (mjg_pid_real)0;
    pid->prev_input      = (mjg_pid_real)0;
    pid->prev_derivative = (mjg_pid_real)0;
    pid->primed          = false;

    /* Zero is not always inside the output range, so land on the nearest
     * value that is. */
    pid->output = clamp((mjg_pid_real)0, pid->cfg.out_min, pid->cfg.out_max);
}

bool mjg_pid_init(mjg_pid *pid, const mjg_pid_config *cfg)
{
    if (pid == NULL || !mjg_pid_validate(cfg)) {
        return false;
    }

    pid->cfg = *cfg;
    mjg_pid_reset(pid);

    return true;
}

void mjg_pid_preload(mjg_pid *pid, mjg_pid_real output)
{
    if (pid == NULL) {
        return;
    }

    mjg_pid_reset(pid);
    pid->output = clamp(output, pid->cfg.out_min, pid->cfg.out_max);

    /* With no error and no derivative, out = ki*integral, so this is the
     * integral that reproduces the requested output. */
    if (pid->cfg.ki != (mjg_pid_real)0) {
        pid->integral = clamp(pid->output / pid->cfg.ki,
                              pid->cfg.integral_min,
                              pid->cfg.integral_max);
    }
}

mjg_pid_real mjg_pid_update(mjg_pid *pid, mjg_pid_real setpoint,
                            mjg_pid_real measurement, mjg_pid_real dt)
{
    mjg_pid_real error;
    mjg_pid_real rate;
    mjg_pid_real derivative;
    mjg_pid_real output;
    bool         wound_up;

    if (pid == NULL) {
        return (mjg_pid_real)0;
    }
    if (dt <= (mjg_pid_real)0) {
        return pid->output;
    }

    error = setpoint - measurement;

    /* Rate of change. Zero on the first update, because there is no previous
     * sample to difference against and a cold start must not spike. */
    if (!pid->primed) {
        rate = (mjg_pid_real)0;
    } else if (pid->cfg.derivative_on_measurement) {
        rate = -(measurement - pid->prev_input) / dt;
    } else {
        rate = (error - pid->prev_input) / dt;
    }

    derivative = pid->cfg.kd * rate;
    derivative = ((mjg_pid_real)1 - pid->cfg.d_filter) * pid->prev_derivative
                 + pid->cfg.d_filter * derivative;

    /* Conditional integration. The output being tested is the previous one,
     * because this update's output is not known yet. */
    wound_up = (pid->output >= pid->cfg.out_max && error > (mjg_pid_real)0) ||
               (pid->output <= pid->cfg.out_min && error < (mjg_pid_real)0);

    if (!wound_up) {
        pid->integral = clamp(pid->integral + (error * dt),
                              pid->cfg.integral_min,
                              pid->cfg.integral_max);
    }

    output = (pid->cfg.kp * error) + (pid->cfg.ki * pid->integral) + derivative;
    output = clamp(output, pid->cfg.out_min, pid->cfg.out_max);

    pid->prev_input      = pid->cfg.derivative_on_measurement ? measurement : error;
    pid->prev_derivative = derivative;
    pid->output          = output;
    pid->primed          = true;

    return output;
}

mjg_pid_real mjg_pid_output(const mjg_pid *pid)
{
    if (pid == NULL) {
        return (mjg_pid_real)0;
    }
    return pid->output;
}
