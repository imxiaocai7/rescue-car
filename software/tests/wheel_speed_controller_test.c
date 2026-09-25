#include "wheel_speed_controller.h"

#include <assert.h>
#include <math.h>
#include <stdio.h>

static wheel_speed_controller_config_t test_config(void) {
    wheel_speed_controller_config_t config = {
        1000.0F,
        100.0F,
        1.0F,
        2.0F,
        500.0F,
        0.1F
    };
    return config;
}

static void test_slew_limit_and_pi_output(void) {
    wheel_speed_controller_t controller;
    wheel_speed_controller_config_t config = test_config();
    float duty = -1.0F;
    assert(wheel_speed_controller_init(&controller, &config));

    assert(wheel_speed_controller_update(&controller, 1.0F, 0.0F, 0.1F, true, &duty));
    assert(fabsf(controller.setpoint_m_s - 0.2F) < 0.0001F);
    assert(fabsf(duty - 202.0F) < 0.001F);
    assert(wheel_speed_controller_update(&controller, 1.0F, 0.0F, 0.1F, true, &duty));
    assert(fabsf(controller.setpoint_m_s - 0.4F) < 0.0001F);
    assert(duty > 0.0F && duty <= config.max_duty_permille);
}

static void test_saturation_does_not_wind_up(void) {
    wheel_speed_controller_t controller;
    wheel_speed_controller_config_t config = test_config();
    float duty = 0.0F;
    config.kp_permille_per_m_s = 1000.0F;
    config.ki_permille_per_m_s_s = 1000.0F;
    config.max_speed_m_s = 0.5F;
    config.max_acceleration_m_s2 = 100.0F;
    config.max_duty_permille = 100.0F;
    assert(wheel_speed_controller_init(&controller, &config));

    assert(wheel_speed_controller_update(&controller, 0.5F, 0.0F, 0.1F, true, &duty));
    assert(duty == config.max_duty_permille);
    assert(controller.integral_permille == 0.0F);
    assert(wheel_speed_controller_update(&controller, 0.5F, 0.5F, 0.1F, true, &duty));
    assert(duty == 0.0F);
}

static void test_disable_and_invalid_dt_are_fail_safe(void) {
    wheel_speed_controller_t controller;
    wheel_speed_controller_config_t config = test_config();
    float duty = 0.0F;
    assert(wheel_speed_controller_init(&controller, &config));
    assert(wheel_speed_controller_update(&controller, 0.5F, 0.0F, 0.05F, true, &duty));
    assert(duty > 0.0F);

    assert(wheel_speed_controller_update(&controller, 0.5F, 0.2F, 0.05F, false, &duty));
    assert(duty == 0.0F);
    assert(controller.setpoint_m_s == 0.0F);
    assert(controller.integral_permille == 0.0F);

    assert(!wheel_speed_controller_update(&controller, 0.5F, 0.0F, 0.2F, true, &duty));
    assert(duty == 0.0F);
    assert(controller.setpoint_m_s == 0.0F);
}

static void test_config_and_target_bounds(void) {
    wheel_speed_controller_t controller;
    wheel_speed_controller_config_t config = test_config();
    float duty = 0.0F;
    config.max_duty_permille = 1001.0F;
    assert(!wheel_speed_controller_init(&controller, &config));

    config = test_config();
    assert(wheel_speed_controller_init(&controller, &config));
    assert(wheel_speed_controller_update(&controller, 5.0F, 0.0F, 0.1F, true, &duty));
    assert(controller.setpoint_m_s <= config.max_speed_m_s);
    assert(duty <= config.max_duty_permille);
}

int main(void) {
    test_slew_limit_and_pi_output();
    test_saturation_does_not_wind_up();
    test_disable_and_invalid_dt_are_fail_safe();
    test_config_and_target_bounds();
    puts("wheel_speed_controller_test: all tests passed");
    return 0;
}
