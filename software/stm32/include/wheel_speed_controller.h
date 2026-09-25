#ifndef RESCUE_WHEEL_SPEED_CONTROLLER_H
#define RESCUE_WHEEL_SPEED_CONTROLLER_H

#include <stdbool.h>

typedef struct {
    float kp_permille_per_m_s;
    float ki_permille_per_m_s_s;
    float max_speed_m_s;
    float max_acceleration_m_s2;
    float max_duty_permille;
    float max_dt_s;
} wheel_speed_controller_config_t;

typedef struct {
    wheel_speed_controller_config_t config;
    float setpoint_m_s;
    float integral_permille;
    bool initialized;
} wheel_speed_controller_t;

/* Gains and limits are mandatory calibration inputs; this module has no defaults. */
bool wheel_speed_controller_init(
    wheel_speed_controller_t *controller,
    const wheel_speed_controller_config_t *config);

/*
 * Calculate signed PWM duty in [-max_duty_permille, +max_duty_permille].
 * Disabling is an immediate zero-output/reset path for expired commands.
 * Invalid measurements or an excessive dt also reset state and return false.
 */
bool wheel_speed_controller_update(
    wheel_speed_controller_t *controller,
    float target_m_s,
    float measured_m_s,
    float dt_s,
    bool enabled,
    float *duty_permille);

void wheel_speed_controller_reset(wheel_speed_controller_t *controller);

#endif
