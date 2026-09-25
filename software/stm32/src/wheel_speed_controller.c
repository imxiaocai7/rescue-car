#include "wheel_speed_controller.h"

#include <math.h>
#include <stddef.h>

static bool valid_config(const wheel_speed_controller_config_t *config) {
    if (config == NULL) {
        return false;
    }
    if (!isfinite(config->kp_permille_per_m_s)
        || !isfinite(config->ki_permille_per_m_s_s)
        || !isfinite(config->max_speed_m_s)
        || !isfinite(config->max_acceleration_m_s2)
        || !isfinite(config->max_duty_permille)
        || !isfinite(config->max_dt_s)) {
        return false;
    }
    return config->kp_permille_per_m_s >= 0.0F
        && config->ki_permille_per_m_s_s >= 0.0F
        && (config->kp_permille_per_m_s > 0.0F || config->ki_permille_per_m_s_s > 0.0F)
        && config->max_speed_m_s > 0.0F
        && config->max_acceleration_m_s2 > 0.0F
        && config->max_duty_permille > 0.0F
        && config->max_duty_permille <= 1000.0F
        && config->max_dt_s > 0.0F;
}

static float clamp(float value, float minimum, float maximum) {
    if (value < minimum) {
        return minimum;
    }
    if (value > maximum) {
        return maximum;
    }
    return value;
}

void wheel_speed_controller_reset(wheel_speed_controller_t *controller) {
    if (controller == NULL) {
        return;
    }
    controller->setpoint_m_s = 0.0F;
    controller->integral_permille = 0.0F;
}

bool wheel_speed_controller_init(
    wheel_speed_controller_t *controller,
    const wheel_speed_controller_config_t *config) {
    if (controller == NULL || !valid_config(config)) {
        return false;
    }
    controller->config = *config;
    controller->setpoint_m_s = 0.0F;
    controller->integral_permille = 0.0F;
    controller->initialized = true;
    return true;
}

bool wheel_speed_controller_update(
    wheel_speed_controller_t *controller,
    float target_m_s,
    float measured_m_s,
    float dt_s,
    bool enabled,
    float *duty_permille) {
    if (duty_permille == NULL) {
        return false;
    }
    *duty_permille = 0.0F;
    if (controller == NULL || !controller->initialized) {
        return false;
    }
    if (!enabled) {
        wheel_speed_controller_reset(controller);
        return true;
    }
    if (!isfinite(target_m_s)
        || !isfinite(measured_m_s)
        || !isfinite(dt_s)
        || dt_s <= 0.0F
        || dt_s > controller->config.max_dt_s) {
        wheel_speed_controller_reset(controller);
        return false;
    }

    const wheel_speed_controller_config_t *config = &controller->config;
    const float bounded_target = clamp(
        target_m_s,
        -config->max_speed_m_s,
        config->max_speed_m_s);
    const float max_setpoint_delta = config->max_acceleration_m_s2 * dt_s;
    if (!isfinite(max_setpoint_delta)) {
        wheel_speed_controller_reset(controller);
        return false;
    }
    controller->setpoint_m_s += clamp(
        bounded_target - controller->setpoint_m_s,
        -max_setpoint_delta,
        max_setpoint_delta);

    const float error_m_s = controller->setpoint_m_s - measured_m_s;
    const float proportional = config->kp_permille_per_m_s * error_m_s;
    const float candidate_integral = clamp(
        controller->integral_permille
            + config->ki_permille_per_m_s_s * error_m_s * dt_s,
        -config->max_duty_permille,
        config->max_duty_permille);
    if (!isfinite(proportional) || !isfinite(candidate_integral)) {
        wheel_speed_controller_reset(controller);
        return false;
    }

    const float candidate_output = proportional + candidate_integral;
    const bool drives_further_into_saturation =
        (candidate_output > config->max_duty_permille && error_m_s > 0.0F)
        || (candidate_output < -config->max_duty_permille && error_m_s < 0.0F);
    if (!drives_further_into_saturation) {
        controller->integral_permille = candidate_integral;
    }

    const float output = proportional + controller->integral_permille;
    if (!isfinite(output)) {
        wheel_speed_controller_reset(controller);
        return false;
    }
    *duty_permille = clamp(
        output,
        -config->max_duty_permille,
        config->max_duty_permille);
    return true;
}
