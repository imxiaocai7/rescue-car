#include "motion_control.h"

#include <assert.h>
#include <stdio.h>

static wheel_speed_controller_config_t test_config(void) {
    wheel_speed_controller_config_t config = {
        1000.0F,
        100.0F,
        0.5F,
        10.0F,
        500.0F,
        0.1F
    };
    return config;
}

static void test_valid_command_drives_both_controllers(void) {
    motion_control_t control;
    wheel_speed_controller_config_t config = test_config();
    motion_command_t command = {1U, 100U, 120, -80, MOTION_ACTUATOR_OPEN};
    motion_control_output_t output;
    assert(motion_control_init(&control, &config, &config));
    assert(motion_control_accept(&control, &command, 1000U));
    assert(motion_control_update(&control, 1050U, 0.0F, 0.0F, 0.05F, &output));
    assert(output.left_duty_permille > 0.0F);
    assert(output.right_duty_permille < 0.0F);
    assert(output.left_duty_permille <= config.max_duty_permille);
    assert(output.right_duty_permille >= -config.max_duty_permille);
    assert(output.actuator == MOTION_ACTUATOR_OPEN);
}

static void test_expiry_replay_and_invalid_feedback_stop_outputs(void) {
    motion_control_t control;
    wheel_speed_controller_config_t config = test_config();
    motion_command_t command = {8U, 20U, 200, 200, MOTION_ACTUATOR_CLOSE};
    motion_control_output_t output;
    assert(motion_control_init(&control, &config, &config));
    assert(motion_control_accept(&control, &command, 200U));
    assert(!motion_control_accept(&control, &command, 201U));
    assert(motion_control_update(&control, 210U, 0.0F, 0.0F, 0.05F, &output));
    assert(output.left_duty_permille > 0.0F);

    assert(!motion_control_update(&control, 220U, 0.0F, 0.0F, 0.05F, &output));
    assert(output.left_duty_permille == 0.0F);
    assert(output.right_duty_permille == 0.0F);
    assert(output.actuator == MOTION_ACTUATOR_HOLD);

    command.sequence = 9U;
    command.lease_ms = 100U;
    assert(motion_control_accept(&control, &command, 230U));
    assert(!motion_control_update(&control, 231U, 0.0F, 0.0F, 0.2F, &output));
    assert(output.left_duty_permille == 0.0F);
    assert(output.right_duty_permille == 0.0F);
    assert(output.actuator == MOTION_ACTUATOR_HOLD);
}

static void test_invalid_configuration_fails_initialization(void) {
    motion_control_t control;
    wheel_speed_controller_config_t valid = test_config();
    wheel_speed_controller_config_t invalid = test_config();
    invalid.max_dt_s = 0.0F;
    assert(!motion_control_init(&control, &valid, &invalid));
    assert(!control.initialized);
}

int main(void) {
    test_valid_command_drives_both_controllers();
    test_expiry_replay_and_invalid_feedback_stop_outputs();
    test_invalid_configuration_fails_initialization();
    puts("motion_control_test: all tests passed");
    return 0;
}
