/*
 * test_pid.c - host tests for mjg_pid.
 *
 * SPDX-License-Identifier: MIT
 * Copyright (c) 2026 Maicon Galiazi
 *
 * No framework: the component must test with nothing but a C99 compiler.
 */

#include <stdio.h>

#include "mjg_pid.h"

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

#define R(x) ((mjg_pid_real)(x))

static bool near(mjg_pid_real a, mjg_pid_real b, mjg_pid_real tol)
{
    mjg_pid_real d = a - b;

    if (d < R(0)) {
        d = -d;
    }
    return d <= tol;
}

/* A config with everything switched off, ready to enable one term at a time. */
static mjg_pid_config plain(void)
{
    mjg_pid_config cfg;

    mjg_pid_defaults(&cfg);
    cfg.kp      = R(0);
    cfg.out_min = R(-1000);
    cfg.out_max = R(1000);

    return cfg;
}

static void test_proportional(void)
{
    mjg_pid_config cfg = plain();
    mjg_pid        pid;

    cfg.kp      = R(2);
    cfg.out_min = R(-100);
    cfg.out_max = R(100);

    CHECK(mjg_pid_init(&pid, &cfg), "init succeeds");
    CHECK(near(mjg_pid_update(&pid, R(10), R(0), R(1)), R(20), R(1e-4)), "output is kp*error");
    CHECK(near(mjg_pid_output(&pid), R(20), R(1e-4)), "and is readable afterwards");

    CHECK(near(mjg_pid_update(&pid, R(500), R(0), R(1)), R(100), R(1e-4)), "clamps at out_max");
    CHECK(near(mjg_pid_update(&pid, R(-500), R(0), R(1)), R(-100), R(1e-4)), "clamps at out_min");
}

static void test_integral_accumulates(void)
{
    mjg_pid_config cfg = plain();
    mjg_pid        pid;

    cfg.ki = R(1);

    CHECK(mjg_pid_init(&pid, &cfg), "init");
    CHECK(near(mjg_pid_update(&pid, R(1), R(0), R(0.5)), R(0.5), R(1e-4)), "integrates error*dt");
    CHECK(near(mjg_pid_update(&pid, R(1), R(0), R(0.5)), R(1.0), R(1e-4)), "and keeps accumulating");
    CHECK(near(mjg_pid_update(&pid, R(1), R(0), R(0.5)), R(1.5), R(1e-4)), "linearly in dt");
}

static void test_integral_limits(void)
{
    mjg_pid_config cfg = plain();
    mjg_pid        pid;

    cfg.ki           = R(1);
    cfg.integral_max = R(2);
    cfg.integral_min = R(-2);

    CHECK(mjg_pid_init(&pid, &cfg), "init");
    mjg_pid_update(&pid, R(10), R(0), R(1));
    mjg_pid_update(&pid, R(10), R(0), R(1));
    CHECK(near(pid.integral, R(2), R(1e-4)), "the integral stops at integral_max");

    mjg_pid_update(&pid, R(-10), R(0), R(1));
    mjg_pid_update(&pid, R(-10), R(0), R(1));
    CHECK(near(pid.integral, R(-2), R(1e-4)), "and at integral_min");
}

static void test_anti_windup(void)
{
    mjg_pid_config cfg = plain();
    mjg_pid        pid;
    mjg_pid_real   held;

    cfg.kp      = R(1);
    cfg.ki      = R(1);
    cfg.out_min = R(-10);
    cfg.out_max = R(10);

    CHECK(mjg_pid_init(&pid, &cfg), "init");

    /* A large constant error drives the output hard into its limit. */
    mjg_pid_update(&pid, R(100), R(0), R(1));
    held = pid.integral;
    CHECK(near(mjg_pid_output(&pid), R(10), R(1e-4)), "the output saturates");

    mjg_pid_update(&pid, R(100), R(0), R(1));
    mjg_pid_update(&pid, R(100), R(0), R(1));
    CHECK(near(pid.integral, held, R(1e-4)), "the integral stops growing while saturated");

    /* Reversing the error must let it move again straight away. */
    mjg_pid_update(&pid, R(-100), R(0), R(1));
    CHECK(pid.integral < held, "and resumes as soon as the error reverses");
}

static void test_derivative_is_zero_on_first_update(void)
{
    mjg_pid_config cfg = plain();
    mjg_pid        pid;

    cfg.kd = R(100);

    CHECK(mjg_pid_init(&pid, &cfg), "init");
    CHECK(near(mjg_pid_update(&pid, R(0), R(50), R(1)), R(0), R(1e-4)),
          "a cold start does not spike, however far off the measurement is");
}

static void test_derivative_on_measurement_has_no_kick(void)
{
    mjg_pid_config cfg = plain();
    mjg_pid        pid;

    cfg.kd                       = R(10);
    cfg.derivative_on_measurement = true;

    CHECK(mjg_pid_init(&pid, &cfg), "init");
    mjg_pid_update(&pid, R(0), R(0), R(1));

    /* The setpoint jumps; the measurement has not moved. */
    CHECK(near(mjg_pid_update(&pid, R(100), R(0), R(1)), R(0), R(1e-4)),
          "a setpoint step produces no derivative kick");

    /* The measurement moves; now the derivative should respond. */
    CHECK(near(mjg_pid_update(&pid, R(100), R(10), R(1)), R(-100), R(1e-4)),
          "but a moving measurement does drive the derivative");
}

static void test_derivative_on_error_does_kick(void)
{
    mjg_pid_config cfg = plain();
    mjg_pid        pid;

    cfg.kd                       = R(10);
    cfg.derivative_on_measurement = false;

    CHECK(mjg_pid_init(&pid, &cfg), "init");
    mjg_pid_update(&pid, R(0), R(0), R(1));

    CHECK(near(mjg_pid_update(&pid, R(100), R(0), R(1)), R(1000), R(1e-3)),
          "on error, the same setpoint step kicks hard");
}

static void test_derivative_filter(void)
{
    mjg_pid_config cfg = plain();
    mjg_pid        pid;

    cfg.kd                       = R(1);
    cfg.d_filter                 = R(0.5);
    cfg.derivative_on_measurement = false;

    CHECK(mjg_pid_init(&pid, &cfg), "init");
    mjg_pid_update(&pid, R(0), R(0), R(1));

    /* Unfiltered this step would be 10; at a weight of 0.5 it moves halfway. */
    CHECK(near(mjg_pid_update(&pid, R(10), R(0), R(1)), R(5), R(1e-4)),
          "the filter moves the derivative only partway");

    cfg.d_filter = R(1);
    CHECK(mjg_pid_init(&pid, &cfg), "re-init unfiltered");
    mjg_pid_update(&pid, R(0), R(0), R(1));
    CHECK(near(mjg_pid_update(&pid, R(10), R(0), R(1)), R(10), R(1e-4)),
          "a weight of 1 leaves it unfiltered");
}

static void test_nonpositive_dt_is_a_no_op(void)
{
    mjg_pid_config cfg = plain();
    mjg_pid        pid;
    mjg_pid_real   before;

    cfg.kp = R(1);
    cfg.ki = R(1);

    CHECK(mjg_pid_init(&pid, &cfg), "init");
    mjg_pid_update(&pid, R(5), R(0), R(1));
    before = pid.integral;

    CHECK(near(mjg_pid_update(&pid, R(5), R(0), R(0)), mjg_pid_output(&pid), R(1e-4)),
          "dt of zero returns the previous output");
    CHECK(near(pid.integral, before, R(1e-4)), "and does not integrate");

    mjg_pid_update(&pid, R(5), R(0), R(-1));
    CHECK(near(pid.integral, before, R(1e-4)), "a negative dt is ignored too");
}

static void test_reset_and_preload(void)
{
    mjg_pid_config cfg = plain();
    mjg_pid        pid;

    cfg.kp = R(1);
    cfg.ki = R(2);

    CHECK(mjg_pid_init(&pid, &cfg), "init");
    mjg_pid_update(&pid, R(10), R(0), R(1));
    CHECK(pid.integral != R(0), "the loop has state to clear");

    mjg_pid_reset(&pid);
    CHECK(near(pid.integral, R(0), R(1e-6)), "reset clears the integral");
    CHECK(near(mjg_pid_output(&pid), R(0), R(1e-6)), "and the output");
    CHECK(pid.primed == false, "and the derivative history");

    /* Handing over from manual control: resume at 50 rather than from zero. */
    mjg_pid_preload(&pid, R(50));
    CHECK(near(mjg_pid_output(&pid), R(50), R(1e-4)), "preload sets the output");
    CHECK(near(mjg_pid_update(&pid, R(0), R(0), R(1)), R(50), R(1e-3)),
          "and the next update with no error holds it there");
}

static void test_validate(void)
{
    mjg_pid_config cfg;

    CHECK(!mjg_pid_validate(NULL), "a NULL config is invalid");

    mjg_pid_defaults(&cfg);
    CHECK(mjg_pid_validate(&cfg), "the defaults are valid");

    mjg_pid_defaults(&cfg);
    cfg.out_min = R(10);
    cfg.out_max = R(10);
    CHECK(!mjg_pid_validate(&cfg), "an empty output range is rejected");

    mjg_pid_defaults(&cfg);
    cfg.out_min = R(20);
    cfg.out_max = R(10);
    CHECK(!mjg_pid_validate(&cfg), "an inverted output range is rejected");

    mjg_pid_defaults(&cfg);
    cfg.integral_min = R(5);
    cfg.integral_max = R(-5);
    CHECK(!mjg_pid_validate(&cfg), "an inverted integral range is rejected");

    mjg_pid_defaults(&cfg);
    cfg.d_filter = R(0);
    CHECK(!mjg_pid_validate(&cfg), "a filter weight of zero is rejected");

    mjg_pid_defaults(&cfg);
    cfg.d_filter = R(1.5);
    CHECK(!mjg_pid_validate(&cfg), "a filter weight above one is rejected");

    mjg_pid_defaults(&cfg);
    cfg.kp = R(-2);
    CHECK(mjg_pid_validate(&cfg), "a negative gain is legal, for a reverse-acting loop");
}

static void test_bad_input_is_inert(void)
{
    mjg_pid_config cfg = plain();
    mjg_pid        pid;

    CHECK(!mjg_pid_init(NULL, &cfg), "a NULL controller is rejected");
    CHECK(!mjg_pid_init(&pid, NULL), "a NULL config is rejected");

    cfg.out_min = R(10);
    cfg.out_max = R(-10);
    CHECK(!mjg_pid_init(&pid, &cfg), "an invalid config is rejected");

    CHECK(near(mjg_pid_update(NULL, R(1), R(0), R(1)), R(0), R(1e-6)),
          "updating a NULL controller returns zero instead of crashing");
    CHECK(near(mjg_pid_output(NULL), R(0), R(1e-6)), "so does reading its output");

    mjg_pid_reset(NULL);
    mjg_pid_preload(NULL, R(1));
    mjg_pid_defaults(NULL);
    CHECK(true, "the NULL-tolerant calls all return instead of crashing");
}

static void test_controllers_are_independent(void)
{
    mjg_pid_config cfg = plain();
    mjg_pid        a, b;

    cfg.ki = R(1);

    CHECK(mjg_pid_init(&a, &cfg), "a inits");
    CHECK(mjg_pid_init(&b, &cfg), "b inits");

    mjg_pid_update(&a, R(10), R(0), R(1));
    mjg_pid_update(&a, R(10), R(0), R(1));

    CHECK(near(a.integral, R(20), R(1e-4)), "a accumulated");
    CHECK(near(b.integral, R(0), R(1e-6)), "b, sharing the config, did not");
}

/* A first-order plant: dy/dt = (u - y) * rate. */
static void test_closed_loop_converges(void)
{
    mjg_pid_config cfg = plain();
    mjg_pid        pid;

    const mjg_pid_real dt       = R(0.01);
    const mjg_pid_real rate     = R(1);
    const mjg_pid_real setpoint = R(10);

    mjg_pid_real y = R(0);
    mjg_pid_real u;
    int          i;

    cfg.kp       = R(1);
    cfg.ki       = R(2);
    cfg.kd       = R(0.05);
    cfg.d_filter = R(0.3);
    cfg.out_min  = R(-100);
    cfg.out_max  = R(100);

    CHECK(mjg_pid_init(&pid, &cfg), "init");

    for (i = 0; i < 2000; ++i) {
        u = mjg_pid_update(&pid, setpoint, y, dt);
        y += (u - y) * rate * dt;
    }

    CHECK(near(y, setpoint, R(0.05)), "the loop drives the plant to setpoint");
}

int main(void)
{
    test_proportional();
    test_integral_accumulates();
    test_integral_limits();
    test_anti_windup();
    test_derivative_is_zero_on_first_update();
    test_derivative_on_measurement_has_no_kick();
    test_derivative_on_error_does_kick();
    test_derivative_filter();
    test_nonpositive_dt_is_a_no_op();
    test_reset_and_preload();
    test_validate();
    test_bad_input_is_inert();
    test_controllers_are_independent();
    test_closed_loop_converges();

    if (failures == 0) {
        printf("ok    %d checks passed\n", checks);
        return 0;
    }

    printf("FAILED %d of %d checks\n", failures, checks);
    return 1;
}
