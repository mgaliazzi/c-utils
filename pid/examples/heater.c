/*
 * heater.c - a worked mjg_pid loop.
 *
 * SPDX-License-Identifier: MIT
 * Copyright (c) 2026 Maicon Galiazi
 *
 * A simulated heater: power in, temperature out, losing heat to ambient.
 * Two things are on show.
 *
 *   - Anti-windup. From a cold start the output pins at 100% for several
 *     seconds. The integrator is frozen for as long as it does, so the
 *     temperature arrives without the overshoot a wound-up integral gives.
 *
 *   - Derivative kick. Two controllers run on the same numbers: the one
 *     driving the plant takes its derivative from the measurement, the
 *     other from the error. Only the second jumps at the setpoint step,
 *     which is the whole reason on-measurement is the default.
 *
 * Build:
 *     cc -std=c99 -I../include ../src/mjg_pid.c heater.c -o heater
 */

#include <stdio.h>

#include "mjg_pid.h"

#define DT          0.1f /* seconds per step  */
#define STEPS       1200 /* 120 seconds       */
#define PRINT_EVERY 25   /* every 2.5 seconds */
#define STEP_AT     500  /* setpoint step at 50 s */

#define AMBIENT   20.0f
#define HEAT_GAIN 0.05f /* degC per second at 100% power      */
#define LOSS      0.05f /* per second, per degC over ambient  */

int main(void)
{
    mjg_pid_config cfg;
    mjg_pid        driving;  /* derivative on measurement; drives the plant */
    mjg_pid        observer; /* derivative on error; watches, changes nothing */

    float temperature = AMBIENT;
    float setpoint    = 60.0f;
    float power       = 0.0f;
    float would_be    = 0.0f;
    int   i;

    mjg_pid_defaults(&cfg);
    cfg.kp       = 5.0f;
    cfg.ki       = 0.5f;
    cfg.kd       = 2.0f;
    cfg.d_filter = 0.2f;
    cfg.out_min  = 0.0f; /* the heater cannot cool */
    cfg.out_max  = 100.0f;

    if (!mjg_pid_init(&driving, &cfg)) {
        printf("the config is invalid\n");
        return 1;
    }

    cfg.derivative_on_measurement = false;
    if (!mjg_pid_init(&observer, &cfg)) {
        printf("the config is invalid\n");
        return 1;
    }

    printf("heater: 120 s, setpoint steps 60 -> 65 degC at t=50 s\n\n");
    printf("   time   set    temp   power   on-error\n");

    for (i = 0; i < STEPS; ++i) {
        float now = (float)i * DT;

        if (i == STEP_AT) {
            setpoint = 65.0f;
            printf("  ---- setpoint step ----\n");
        }

        power    = mjg_pid_update(&driving, setpoint, temperature, DT);
        would_be = mjg_pid_update(&observer, setpoint, temperature, DT);

        /* Plant: heating from the element, cooling toward ambient. */
        temperature += ((power * HEAT_GAIN) - ((temperature - AMBIENT) * LOSS)) * DT;

        if ((i % PRINT_EVERY) == 0) {
            printf("  %5.1fs  %5.1f  %6.2f  %5.1f%%  %5.1f%%\n",
                   now, setpoint, temperature, power, would_be);
        }
    }

    printf("\nsettled at %.2f degC with the setpoint at %.1f\n", temperature, setpoint);

    return 0;
}
