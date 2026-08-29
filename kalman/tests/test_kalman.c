/*
 * test_kalman.c - host tests for mjg_kalman.
 *
 * SPDX-License-Identifier: MIT
 * Copyright (c) 2026 Maicon Galiazi
 *
 * No framework: the component must test with nothing but a C99 compiler.
 */

#include <stdio.h>

#include "mjg_kalman.h"

static int checks   = 0;
static int failures = 0;

#define CHECK(cond, msg)                                            \
    do {                                                            \
        ++checks;                                                   \
        if (!(cond)) {                                              \
            ++failures;                                             \
            printf("FAIL  %s:%d  %s\n", __FILE__, __LINE__, (msg)); \
        }                                                           \
    } while (0)

#define R(x) ((mjg_kalman_real)(x))

static bool near(mjg_kalman_real a, mjg_kalman_real b, mjg_kalman_real tol)
{
    mjg_kalman_real d = a - b;

    if (d < R(0)) {
        d = -d;
    }
    return d <= tol;
}

/* A deterministic generator, so the printed output and the assertions are
 * identical on every platform. Roughly uniform on -1 .. 1. */
static uint32_t rng = 12345u;

static mjg_kalman_real noise(void)
{
    rng = (rng * 1103515245u) + 12345u;
    return (R((rng >> 16) & 0x7FFFu) / R(16383.5)) - R(1);
}

static void test_scalar_trusts_the_model_or_the_sensor(void)
{
    mjg_kalman_config cfg;
    mjg_kalman        kf;
    int               i;

    /* A very noisy sensor: the estimate should barely react. */
    mjg_kalman_model_scalar(&cfg, R(1e-6), R(1e6));
    CHECK(mjg_kalman_init(&kf, &cfg, NULL, NULL), "init");
    for (i = 0; i < 5; ++i) {
        mjg_kalman_step(&kf, R(0), R(100));
    }
    CHECK(mjg_kalman_state(&kf, 0) < R(1), "a huge R keeps the estimate near where it was");

    /* A very uncertain model: the estimate should chase the measurement. */
    mjg_kalman_model_scalar(&cfg, R(1e6), R(1e-6));
    CHECK(mjg_kalman_init(&kf, &cfg, NULL, NULL), "init");
    mjg_kalman_step(&kf, R(0), R(100));
    CHECK(near(mjg_kalman_state(&kf, 0), R(100), R(0.1)),
          "a huge Q makes the estimate follow the measurement");
}

static void test_scalar_gain_reaches_the_known_steady_state(void)
{
    mjg_kalman_config cfg;
    mjg_kalman        kf;
    int               i;

    /* With F=H=1 and Q=R=1 the post-update covariance converges to a value
     * with a closed form: P = (sqrt(5)-1)/2, the reciprocal of the golden
     * ratio, 0.6180... It is an exact number to assert against. */
    mjg_kalman_model_scalar(&cfg, R(1), R(1));
    CHECK(mjg_kalman_init(&kf, &cfg, NULL, NULL), "init");

    for (i = 0; i < 200; ++i) {
        mjg_kalman_step(&kf, R(0), R(0));
    }

    CHECK(near(mjg_kalman_covariance(&kf, 0, 0), R(0.6180339), R(1e-4)),
          "the covariance settles on its closed-form steady state");
}

static void test_velocity_is_recovered_from_position_only(void)
{
    mjg_kalman_config cfg;
    mjg_kalman        kf;
    const mjg_kalman_real dt       = R(0.1);
    const mjg_kalman_real velocity = R(2);
    mjg_kalman_real       position = R(0);
    int                   i;

    mjg_kalman_model_constant_velocity(&cfg, dt, R(0.01), R(0.01));
    CHECK(mjg_kalman_init(&kf, &cfg, NULL, NULL), "init");

    for (i = 0; i < 400; ++i) {
        position += velocity * dt;
        mjg_kalman_step(&kf, R(0), position);
    }

    CHECK(near(mjg_kalman_state(&kf, 1), velocity, R(0.01)),
          "velocity converges to truth though only position was ever measured");
    CHECK(near(mjg_kalman_state(&kf, 0), position, R(0.01)), "and position tracks");
}

static void test_predict_grows_p_and_update_shrinks_it(void)
{
    mjg_kalman_config cfg;
    mjg_kalman        kf;
    mjg_kalman_real   after_init, after_predict, after_update;

    mjg_kalman_model_scalar(&cfg, R(0.5), R(1));
    CHECK(mjg_kalman_init(&kf, &cfg, NULL, NULL), "init");

    after_init = mjg_kalman_covariance(&kf, 0, 0);
    mjg_kalman_predict(&kf, R(0));
    after_predict = mjg_kalman_covariance(&kf, 0, 0);
    mjg_kalman_update(&kf, R(0));
    after_update = mjg_kalman_covariance(&kf, 0, 0);

    CHECK(after_predict > after_init, "predicting adds uncertainty");
    CHECK(after_update < after_predict, "measuring removes it");
}

static void test_control_input(void)
{
    mjg_kalman_config cfg;
    mjg_kalman        driven, quiet;
    uint8_t           i, j;
    bool              same = true;

    mjg_kalman_model_scalar(&cfg, R(0.1), R(1));

    /* A zero b[] is what the model builders leave behind, so u must be inert. */
    CHECK(mjg_kalman_init(&driven, &cfg, NULL, NULL), "init");
    mjg_kalman_predict(&driven, R(99));
    CHECK(near(mjg_kalman_state(&driven, 0), R(0), R(1e-6)),
          "with b all zero, a control input does nothing");

    /* Now let the input move the state directly. */
    cfg.b[0] = R(1);
    CHECK(mjg_kalman_init(&driven, &cfg, NULL, NULL), "init");
    mjg_kalman_predict(&driven, R(3));
    CHECK(near(mjg_kalman_state(&driven, 0), R(3), R(1e-6)),
          "one predict moves the state by exactly b*u");

    /* A commanded input is known exactly, so it must not touch P. */
    CHECK(mjg_kalman_init(&driven, &cfg, NULL, NULL), "init");
    CHECK(mjg_kalman_init(&quiet, &cfg, NULL, NULL), "init");
    mjg_kalman_predict(&driven, R(5));
    mjg_kalman_predict(&quiet, R(0));

    for (i = 0u; i < cfg.n; ++i) {
        for (j = 0u; j < cfg.n; ++j) {
            if (!near(mjg_kalman_covariance(&driven, i, j),
                      mjg_kalman_covariance(&quiet, i, j), R(1e-9))) {
                same = false;
            }
        }
    }
    CHECK(same, "a known input adds no uncertainty, so P is unchanged by u");
    CHECK(!near(mjg_kalman_state(&driven, 0), mjg_kalman_state(&quiet, 0), R(1e-6)),
          "but the state estimate did move");
}

static void test_covariance_stays_healthy(void)
{
    mjg_kalman_config cfg;
    mjg_kalman        kf;
    mjg_kalman_real   truth = R(0);
    bool              symmetric = true;
    bool              positive  = true;
    int               i;

    mjg_kalman_model_constant_velocity(&cfg, R(0.05), R(0.5), R(4));
    CHECK(mjg_kalman_init(&kf, &cfg, NULL, NULL), "init");

    /* The short form P = (I-KH)P survives a few hundred steps and then rots.
     * Joseph form plus symmetrisation has to hold up over a long run. */
    for (i = 0; i < 10000; ++i) {
        truth += R(0.05);
        mjg_kalman_step(&kf, R(0), truth + (noise() * R(2)));

        if (!near(mjg_kalman_covariance(&kf, 0, 1), mjg_kalman_covariance(&kf, 1, 0), R(1e-12))) {
            symmetric = false;
        }
        if (mjg_kalman_covariance(&kf, 0, 0) <= R(0) ||
            mjg_kalman_covariance(&kf, 1, 1) <= R(0)) {
            positive = false;
        }
    }

    CHECK(symmetric, "the covariance stays symmetric over 10000 cycles");
    CHECK(positive, "and its diagonal stays positive");
}

static void test_outlier_gating(void)
{
    mjg_kalman_config     cfg;
    mjg_kalman            kf;
    mjg_kalman_real       before;
    const mjg_kalman_real p0[1] = { R(100) }; /* honest about knowing nothing yet */
    int                   i;

    mjg_kalman_model_scalar(&cfg, R(0.01), R(1));
    cfg.gate = R(9); /* three sigma */

    CHECK(mjg_kalman_init(&kf, &cfg, NULL, p0), "init");
    for (i = 0; i < 50; ++i) {
        CHECK(mjg_kalman_step(&kf, R(0), R(10)), "ordinary measurements are accepted");
    }

    before = mjg_kalman_state(&kf, 0);
    CHECK(!mjg_kalman_step(&kf, R(0), R(10000)), "a wild reading is rejected");
    CHECK(near(mjg_kalman_state(&kf, 0), before, R(1e-4)),
          "and leaves the estimate alone");

    /* The same spike with gating off must be swallowed whole. */
    cfg.gate = R(0);
    CHECK(mjg_kalman_init(&kf, &cfg, NULL, NULL), "re-init ungated");
    for (i = 0; i < 50; ++i) {
        mjg_kalman_step(&kf, R(0), R(10));
    }
    before = mjg_kalman_state(&kf, 0);
    CHECK(mjg_kalman_step(&kf, R(0), R(10000)), "with gate = 0 it is accepted");
    CHECK(mjg_kalman_state(&kf, 0) > before + R(1), "and drags the estimate with it");
}

static void test_gating_needs_an_honest_initial_covariance(void)
{
    mjg_kalman_config     cfg;
    mjg_kalman            kf;
    const mjg_kalman_real tight[1] = { R(0.01) };
    int                   i, rejected = 0;

    /* The gate judges a reading against how sure the filter thought it was,
     * and P0 is where that confidence comes from. Claim certainty you do not
     * have and every real measurement looks like an outlier. P does creep up
     * through Q each predict, so it is recoverable in principle - but at this
     * Q it would take on the order of a million steps. */
    mjg_kalman_model_scalar(&cfg, R(0.001), R(1));
    cfg.gate = R(9);

    CHECK(mjg_kalman_init(&kf, &cfg, NULL, tight), "init claiming a tight start");

    for (i = 0; i < 20; ++i) {
        if (!mjg_kalman_step(&kf, R(0), R(100))) {
            ++rejected;
        }
    }

    CHECK(rejected == 20, "an overconfident start makes the gate reject everything");
    CHECK(near(mjg_kalman_state(&kf, 0), R(0), R(1e-3)), "so the filter never converges");
}

static void test_model_builders(void)
{
    mjg_kalman_config cfg;

    mjg_kalman_model_scalar(&cfg, R(2), R(3));
    CHECK(cfg.n == 1u, "the scalar model has one state");
    CHECK(near(cfg.f[0][0], R(1), R(1e-9)), "and holds its value");
    CHECK(near(cfg.h[0], R(1), R(1e-9)), "measured directly");
    CHECK(near(cfg.q[0][0], R(2), R(1e-9)), "process variance passed through");
    CHECK(near(cfg.r, R(3), R(1e-9)), "measurement variance passed through");
    CHECK(near(cfg.b[0], R(0), R(1e-9)), "and no control input by default");

    mjg_kalman_model_constant_velocity(&cfg, R(0.5), R(1), R(4));
    CHECK(cfg.n == 2u, "the constant-velocity model has two states");
    CHECK(near(cfg.f[0][0], R(1), R(1e-9)), "F is [[1, dt], [0, 1]]");
    CHECK(near(cfg.f[0][1], R(0.5), R(1e-9)), "with dt in the corner");
    CHECK(near(cfg.f[1][0], R(0), R(1e-9)), "velocity does not depend on position");
    CHECK(near(cfg.f[1][1], R(1), R(1e-9)), "and carries forward");
    CHECK(near(cfg.h[0], R(1), R(1e-9)), "position is measured");
    CHECK(near(cfg.h[1], R(0), R(1e-9)), "velocity is not");
    CHECK(near(cfg.q[0][1], cfg.q[1][0], R(1e-9)), "Q is symmetric");
    CHECK(near(cfg.q[1][1], R(0.25), R(1e-9)), "and its velocity term is var*dt*dt");
}

static void test_validate(void)
{
    mjg_kalman_config cfg;

    CHECK(!mjg_kalman_validate(NULL), "a NULL config is invalid");

    mjg_kalman_model_scalar(&cfg, R(1), R(1));
    CHECK(mjg_kalman_validate(&cfg), "the control case is valid");

    cfg.n = 0u;
    CHECK(!mjg_kalman_validate(&cfg), "zero states is rejected");

    cfg.n = (uint8_t)(MJG_KALMAN_MAX_STATES + 1);
    CHECK(!mjg_kalman_validate(&cfg), "more states than the build allows is rejected");

    mjg_kalman_model_scalar(&cfg, R(1), R(0));
    CHECK(!mjg_kalman_validate(&cfg), "a zero measurement variance is rejected");

    mjg_kalman_model_scalar(&cfg, R(1), R(-1));
    CHECK(!mjg_kalman_validate(&cfg), "a negative one too");

    mjg_kalman_model_scalar(&cfg, R(1), R(1));
    cfg.gate = R(-1);
    CHECK(!mjg_kalman_validate(&cfg), "a negative gate is rejected");
}

static void test_bad_input_is_inert(void)
{
    mjg_kalman_config cfg;
    mjg_kalman        kf;

    mjg_kalman_model_scalar(&cfg, R(1), R(1));

    CHECK(!mjg_kalman_init(NULL, &cfg, NULL, NULL), "a NULL filter is rejected");
    CHECK(!mjg_kalman_init(&kf, NULL, NULL, NULL), "a NULL config is rejected");

    cfg.r = R(-1);
    CHECK(!mjg_kalman_init(&kf, &cfg, NULL, NULL), "an invalid config is rejected");

    /* Everything must be a no-op on the inert filter, not a wild read. */
    mjg_kalman_predict(&kf, R(1));
    CHECK(!mjg_kalman_update(&kf, R(1)), "updating an inert filter fails cleanly");
    CHECK(near(mjg_kalman_state(&kf, 0), R(0), R(1e-9)), "and it reports no state");
    CHECK(near(mjg_kalman_covariance(&kf, 0, 0), R(0), R(1e-9)), "nor any covariance");

    mjg_kalman_predict(NULL, R(1));
    CHECK(!mjg_kalman_update(NULL, R(1)), "a NULL filter fails cleanly");
    CHECK(near(mjg_kalman_state(NULL, 0), R(0), R(1e-9)), "and reads back as zero");
    CHECK(near(mjg_kalman_innovation(NULL), R(0), R(1e-9)), "including the innovation");
    mjg_kalman_model_scalar(NULL, R(1), R(1));
    mjg_kalman_model_constant_velocity(NULL, R(1), R(1), R(1));
    CHECK(true, "the NULL-tolerant calls all return instead of crashing");
}

static void test_out_of_range_accessors(void)
{
    mjg_kalman_config cfg;
    mjg_kalman        kf;

    mjg_kalman_model_scalar(&cfg, R(1), R(1));
    CHECK(mjg_kalman_init(&kf, &cfg, NULL, NULL), "init");

    CHECK(near(mjg_kalman_state(&kf, 7), R(0), R(1e-9)), "state index past n reads zero");
    CHECK(near(mjg_kalman_covariance(&kf, 7, 0), R(0), R(1e-9)), "so does a covariance row");
    CHECK(near(mjg_kalman_covariance(&kf, 0, 7), R(0), R(1e-9)), "and a covariance column");
}

static void test_filters_are_independent(void)
{
    mjg_kalman_config     cfg;
    mjg_kalman            a, b;
    const mjg_kalman_real x0[1] = { R(0) };

    mjg_kalman_model_scalar(&cfg, R(1), R(1));
    CHECK(mjg_kalman_init(&a, &cfg, x0, NULL), "a inits");
    CHECK(mjg_kalman_init(&b, &cfg, x0, NULL), "b inits");

    mjg_kalman_step(&a, R(0), R(50));
    mjg_kalman_step(&a, R(0), R(50));

    CHECK(mjg_kalman_state(&a, 0) > R(10), "a moved toward its measurements");
    CHECK(near(mjg_kalman_state(&b, 0), R(0), R(1e-9)),
          "b, sharing the same const config, did not");
}

static void test_initial_state_and_covariance(void)
{
    mjg_kalman_config     cfg;
    mjg_kalman            kf;
    const mjg_kalman_real x0[2] = { R(7), R(-3) };
    const mjg_kalman_real p0[2] = { R(9), R(16) };

    mjg_kalman_model_constant_velocity(&cfg, R(0.1), R(1), R(1));
    CHECK(mjg_kalman_init(&kf, &cfg, x0, p0), "init with an explicit start");

    CHECK(near(mjg_kalman_state(&kf, 0), R(7), R(1e-9)), "the initial state is taken");
    CHECK(near(mjg_kalman_state(&kf, 1), R(-3), R(1e-9)), "for every element");
    CHECK(near(mjg_kalman_covariance(&kf, 0, 0), R(9), R(1e-9)), "as is the diagonal");
    CHECK(near(mjg_kalman_covariance(&kf, 1, 1), R(16), R(1e-9)), "of the covariance");
    CHECK(near(mjg_kalman_covariance(&kf, 0, 1), R(0), R(1e-9)), "off-diagonals start at zero");
}

int main(void)
{
    test_scalar_trusts_the_model_or_the_sensor();
    test_scalar_gain_reaches_the_known_steady_state();
    test_velocity_is_recovered_from_position_only();
    test_predict_grows_p_and_update_shrinks_it();
    test_control_input();
    test_covariance_stays_healthy();
    test_outlier_gating();
    test_gating_needs_an_honest_initial_covariance();
    test_model_builders();
    test_validate();
    test_bad_input_is_inert();
    test_out_of_range_accessors();
    test_filters_are_independent();
    test_initial_state_and_covariance();

    if (failures == 0) {
        printf("ok    %d checks passed\n", checks);
        return 0;
    }

    printf("FAILED %d of %d checks\n", failures, checks);
    return 1;
}
