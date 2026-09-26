#ifndef RESCUE_MOTION_CONTROL_H
#define RESCUE_MOTION_CONTROL_H

#include "motion_command_guard.h"
#include "wheel_speed_controller.h"

typedef struct {
    float left_duty_permille;
    float right_duty_permille;
    motion_actuator_command_t actuator;
} motion_control_output_t;

typedef struct {
    motion_command_guard_t command_guard;
    wheel_speed_controller_t left_controller;
    wheel_speed_controller_t right_controller;
    bool initialized;
} motion_control_t;

bool motion_control_init(
    motion_control_t *control,
    const wheel_speed_controller_config_t *left_config,
    const wheel_speed_controller_config_t *right_config);

/* Accepts only decoded, CRC-valid commands; the guard rejects replay/old sequence numbers. */
bool motion_control_accept(
    motion_control_t *control,
    const motion_command_t *command,
    uint32_t received_at_ms);

/* Stop immediately and require a newer command before motion can resume. */
void motion_control_disarm(motion_control_t *control);

/* Returns true only when a fresh command and valid feedback produced outputs. */
bool motion_control_update(
    motion_control_t *control,
    uint32_t now_ms,
    float left_measured_m_s,
    float right_measured_m_s,
    float dt_s,
    motion_control_output_t *output);

#endif
