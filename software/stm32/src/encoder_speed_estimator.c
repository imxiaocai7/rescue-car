#include "encoder_speed_estimator.h"

#include <math.h>
#include <stddef.h>

#define ENCODER_TAU 6.2831853071795864769
#define UINT32_MODULUS 4294967296LL
#define UINT32_HALF_RANGE 2147483648LL

static bool valid_config(const encoder_speed_estimator_config_t *config) {
    if (config == NULL
        || !isfinite(config->wheel_radius_m)
        || !isfinite(config->max_speed_m_s)
        || !isfinite(config->max_dt_s)) {
        return false;
    }
    return config->wheel_radius_m > 0.0F
        && config->counts_per_wheel_revolution > 0U
        && (config->counter_modulus == 0U || config->counter_modulus >= 2U)
        && config->max_speed_m_s > 0.0F
        && config->max_dt_s > 0.0F;
}

static bool counter_is_valid(
    const encoder_speed_estimator_t *estimator,
    uint32_t counter_count) {
    return estimator->config.counter_modulus == 0U
        || counter_count < estimator->config.counter_modulus;
}

static bool modular_delta(
    const encoder_speed_estimator_config_t *config,
    uint32_t previous,
    uint32_t current,
    int64_t *delta) {
    int64_t difference = (int64_t)current - (int64_t)previous;
    if (config->counter_modulus == 0U) {
        if (difference == UINT32_HALF_RANGE || difference == -UINT32_HALF_RANGE) {
            return false;
        }
        if (difference > UINT32_HALF_RANGE) {
            difference -= UINT32_MODULUS;
        } else if (difference < -UINT32_HALF_RANGE) {
            difference += UINT32_MODULUS;
        }
    } else {
        const int64_t modulus = (int64_t)config->counter_modulus;
        const int64_t half_range = modulus / 2LL;
        if ((modulus % 2LL) == 0LL
            && (difference == half_range || difference == -half_range)) {
            return false;
        }
        if (difference > half_range) {
            difference -= modulus;
        } else if (difference < -half_range) {
            difference += modulus;
        }
    }
    *delta = difference;
    return true;
}

bool encoder_speed_estimator_init(
    encoder_speed_estimator_t *estimator,
    const encoder_speed_estimator_config_t *config) {
    if (estimator == NULL || !valid_config(config)) {
        return false;
    }
    estimator->config = *config;
    estimator->previous_count = 0U;
    estimator->has_previous_count = false;
    estimator->initialized = true;
    return true;
}

bool encoder_speed_estimator_reset(
    encoder_speed_estimator_t *estimator,
    uint32_t counter_count) {
    if (estimator == NULL || !estimator->initialized
        || !counter_is_valid(estimator, counter_count)) {
        return false;
    }
    estimator->previous_count = counter_count;
    estimator->has_previous_count = true;
    return true;
}

bool encoder_speed_estimator_update(
    encoder_speed_estimator_t *estimator,
    uint32_t counter_count,
    float dt_s,
    float *speed_m_s) {
    if (speed_m_s == NULL) {
        return false;
    }
    *speed_m_s = 0.0F;
    if (estimator == NULL || !estimator->initialized
        || !counter_is_valid(estimator, counter_count)
        || !estimator->has_previous_count) {
        return false;
    }

    const uint32_t previous_count = estimator->previous_count;
    estimator->previous_count = counter_count;
    if (!isfinite(dt_s) || dt_s <= 0.0F || dt_s > estimator->config.max_dt_s) {
        return false;
    }

    int64_t count_delta = 0;
    if (!modular_delta(
            &estimator->config,
            previous_count,
            counter_count,
            &count_delta)) {
        return false;
    }

    const double metres_per_count =
        (ENCODER_TAU * (double)estimator->config.wheel_radius_m)
        / (double)estimator->config.counts_per_wheel_revolution;
    const double speed =
        ((double)count_delta * metres_per_count) / (double)dt_s;
    const double one_count_speed = metres_per_count / (double)dt_s;
    if (!isfinite(speed)
        || !isfinite(one_count_speed)
        || fabs(speed) > (double)estimator->config.max_speed_m_s + one_count_speed) {
        return false;
    }

    const float result = (float)speed;
    if (!isfinite(result)) {
        return false;
    }
    *speed_m_s = result;
    return true;
}
