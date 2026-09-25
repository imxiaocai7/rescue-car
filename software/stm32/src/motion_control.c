#include "motion_control.h"

#include <stddef.h>
#include <string.h>

static void stop_output(motion_control_t *control, motion_control_output_t *output) {
    output->left_duty_permille = 0.0F;
    output->right_duty_permille = 0.0F;
    output->actuator = MOTION_ACTUATOR_HOLD;
    wheel_speed_controller_update(
        &control->left_controller, 0.0F, 0.0F, 0.0F, false,
        &output->left_duty_permille);
    wheel_speed_controller_update(
        &control->right_controller, 0.0F, 0.0F, 0.0F, false,
        &output->right_duty_permille);
}

bool motion_control_init(
    motion_control_t *control,
    const wheel_speed_controller_config_t *left_config,
    const wheel_speed_controller_config_t *right_config) {
    if (control == NULL) {
        return false;
    }

    motion_control_t candidate;
    memset(&candidate, 0, sizeof(candidate));
    motion_command_guard_init(&candidate.command_guard);
    if (!wheel_speed_controller_init(&candidate.left_controller, left_config)
        || !wheel_speed_controller_init(&candidate.right_controller, right_config)) {
        memset(control, 0, sizeof(*control));
        return false;
    }
    candidate.initialized = true;
    *control = candidate;
    return true;
}

bool motion_control_accept(
    motion_control_t *control,
    const motion_command_t *command,
    uint32_t received_at_ms) {
    if (control == NULL || !control->initialized) {
        return false;
    }
    return motion_command_guard_accept(
        &control->command_guard, command, received_at_ms);
}

bool motion_control_update(
    motion_control_t *control,
    uint32_t now_ms,
    float left_measured_m_s,
    float right_measured_m_s,
    float dt_s,
    motion_control_output_t *output) {
    if (output == NULL) {
        return false;
    }
    output->left_duty_permille = 0.0F;
    output->right_duty_permille = 0.0F;
    output->actuator = MOTION_ACTUATOR_HOLD;
    if (control == NULL || !control->initialized) {
        return false;
    }

    motion_command_t command;
    if (!motion_command_guard_get(&control->command_guard, now_ms, &command)) {
        stop_output(control, output);
        return false;
    }

    const bool left_ok = wheel_speed_controller_update(
        &control->left_controller,
        (float)command.left_mm_s / 1000.0F,
        left_measured_m_s,
        dt_s,
        true,
        &output->left_duty_permille);
    const bool right_ok = wheel_speed_controller_update(
        &control->right_controller,
        (float)command.right_mm_s / 1000.0F,
        right_measured_m_s,
        dt_s,
        true,
        &output->right_duty_permille);
    if (!left_ok || !right_ok) {
        stop_output(control, output);
        return false;
    }

    output->actuator = command.actuator;
    return true;
}
