/*
 * observer.c - why the control input earns its place.
 *
 * SPDX-License-Identifier: MIT
 * Copyright (c) 2026 Maicon Galiazi
 *
 * A plant is driven by a commanded input and watched through a slow, noisy
 * sensor. Two filters run on exactly the same measurements:
 *
 *   informed    told what was commanded (b is set, u is passed)
 *   uninformed  told nothing (b is zero)
 *
 * The uninformed filter sees the plant move and has no way to explain it, so
 * it must be dragged there by measurements it barely trusts. Watch the two
 * estimates part company right after the setpoint step, and the informed one
 * arrive first.
 *
 * The controller here is a deliberately trivial proportional one, written
 * inline. This example links nothing but mjg_kalman: components in c-utils
 * never depend on each other. See the README for pairing it with pid.
 *
 * Build:
 *     cc -std=c99 -I../include ../src/mjg_kalman.c observer.c -o observer
 */

#include <stdio.h>

#include "mjg_kalman.h"

#define STEPS       60
#define PRINT_EVERY 3
#define STEP_AT     20

#define PLANT_DECAY 0.9f /* what survives of the state each step  */
#define PLANT_GAIN  1.0f /* how hard the input pushes it          */
#define SENSOR_SPAN 2.0f /* the sensor is genuinely bad           */
#define CONTROL_KP  0.4f

static uint32_t rng = 88675123u;

static mjg_kalman_real noise(void)
{
    rng = (rng * 1103515245u) + 12345u;
    return ((mjg_kalman_real)((rng >> 16) & 0x7FFFu) / (mjg_kalman_real)16383.5) -
           (mjg_kalman_real)1;
}

int main(void)
{
    mjg_kalman_config informed_cfg, uninformed_cfg;
    mjg_kalman        informed, uninformed;

    mjg_kalman_real truth    = 0;
    mjg_kalman_real setpoint = 0;
    mjg_kalman_real u        = 0;
    mjg_kalman_real measured;
    int   i;

    /* One state: the plant decays toward zero unless driven. */
    mjg_kalman_model_scalar(&informed_cfg, 0.05f, SENSOR_SPAN * SENSOR_SPAN);
    informed_cfg.f[0][0] = PLANT_DECAY;
    informed_cfg.b[0]    = PLANT_GAIN; /* the filter knows what the input does */

    uninformed_cfg    = informed_cfg;
    uninformed_cfg.b[0] = 0; /* ... and this one does not */

    if (!mjg_kalman_init(&informed, &informed_cfg, NULL, NULL) ||
        !mjg_kalman_init(&uninformed, &uninformed_cfg, NULL, NULL)) {
        printf("the model is invalid\n");
        return 1;
    }

    printf("observer: setpoint steps 0 -> 10 at step %d\n\n", STEP_AT);
    printf("  step  setpoint   truth   measured   informed   uninformed\n");

    for (i = 0; i < STEPS; ++i) {
        if (i == STEP_AT) {
            setpoint = (mjg_kalman_real)10;
            printf("  ---- setpoint step ----\n");
        }

        /* Close the loop on the informed estimate. */
        u = CONTROL_KP * (setpoint - mjg_kalman_state(&informed, 0));

        /* Plant, then a bad look at it. */
        truth    = (PLANT_DECAY * truth) + (PLANT_GAIN * u);
        measured = truth + (noise() * SENSOR_SPAN);

        /* Same measurement to both; only one is told about u. */
        mjg_kalman_step(&informed, u, measured);
        mjg_kalman_step(&uninformed, 0.0f, measured);

        if ((i % PRINT_EVERY) == 0 || (i >= STEP_AT && i < STEP_AT + 6)) {
            printf("  %4d  %8.1f  %6.2f  %9.2f  %9.2f  %11.2f\n",
                   i, (double)setpoint, (double)truth, (double)measured,
                   (double)mjg_kalman_state(&informed, 0),
                   (double)mjg_kalman_state(&uninformed, 0));
        }
    }

    printf("\ntruth %.2f   informed %.2f   uninformed %.2f\n",
           (double)truth,
           (double)mjg_kalman_state(&informed, 0),
           (double)mjg_kalman_state(&uninformed, 0));

    return 0;
}
