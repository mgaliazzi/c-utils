/*
 * tracker.c - a worked mjg_kalman filter.
 *
 * SPDX-License-Identifier: MIT
 * Copyright (c) 2026 Maicon Galiazi
 *
 * An object moving at constant speed, seen through a noisy position sensor.
 * Two things are on show.
 *
 *   - Velocity is recovered although it is never measured. The filter knows
 *     position and velocity are linked, so watching one reveals the other.
 *     A low-pass filter cannot do this.
 *   - One wild reading is injected partway through. The gate rejects it on
 *     the strength of how far it sits from what the model expected.
 *
 * Noise comes from a small generator built in here, so the output is the
 * same on every machine. No rand(), no math.h.
 *
 * Build:
 *     cc -std=c99 -I../include ../src/mjg_kalman.c tracker.c -o tracker
 */

#include <stdio.h>

#include "mjg_kalman.h"

#define DT          0.1f
#define STEPS       200
#define PRINT_EVERY 10
#define OUTLIER_AT  100

#define TRUE_VELOCITY 2.0f
#define NOISE_SPAN    0.8f /* measurements land within about this of truth */

static uint32_t rng = 2463534242u;

/* Roughly uniform on -1 .. 1, deterministic across platforms. */
static mjg_kalman_real noise(void)
{
    rng = (rng * 1103515245u) + 12345u;
    return ((mjg_kalman_real)((rng >> 16) & 0x7FFFu) / (mjg_kalman_real)16383.5) -
           (mjg_kalman_real)1;
}

int main(void)
{
    mjg_kalman_config cfg;
    mjg_kalman        kf;

    mjg_kalman_real position = 0;
    mjg_kalman_real measured;
    int   i;

    /* accel_noise says how much unmodelled acceleration to expect; the
     * measurement variance is the sensor's own spread, squared. */
    mjg_kalman_model_constant_velocity(&cfg, DT, 0.2f, NOISE_SPAN * NOISE_SPAN);
    cfg.gate = 9.0f; /* reject anything past three sigma */

    if (!mjg_kalman_init(&kf, &cfg, NULL, NULL)) {
        printf("the model is invalid\n");
        return 1;
    }

    printf("tracker: %d steps, true velocity %.1f, one outlier at step %d\n\n",
           STEPS, (double)TRUE_VELOCITY, OUTLIER_AT);
    printf("  step    truth    measured    est pos    est vel\n");

    for (i = 0; i < STEPS; ++i) {
        position += TRUE_VELOCITY * DT;
        measured = position + (noise() * NOISE_SPAN);

        if (i == OUTLIER_AT) {
            measured += (mjg_kalman_real)50; /* a sensor glitch */
        }

        if (!mjg_kalman_step(&kf, 0.0f, measured)) {
            printf("  %4d  %7.2f  %10.2f   -- rejected, innovation %.1f --\n",
                   i, (double)position, (double)measured,
                   (double)mjg_kalman_innovation(&kf));
            continue;
        }

        if ((i % PRINT_EVERY) == 0) {
            printf("  %4d  %7.2f  %10.2f  %9.2f  %9.3f\n",
                   i, (double)position, (double)measured,
                   (double)mjg_kalman_state(&kf, 0),
                   (double)mjg_kalman_state(&kf, 1));
        }
    }

    printf("\nfinal velocity estimate %.4f against a true %.1f\n",
           (double)mjg_kalman_state(&kf, 1), (double)TRUE_VELOCITY);

    return 0;
}
