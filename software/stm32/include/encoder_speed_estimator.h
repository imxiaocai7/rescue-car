#ifndef RESCUE_ENCODER_SPEED_ESTIMATOR_H
#define RESCUE_ENCODER_SPEED_ESTIMATOR_H

#include <stdbool.h>
#include <stdint.h>

typedef struct {
    float wheel_radius_m;
    uint32_t counts_per_wheel_revolution;
    uint32_t counter_modulus; /* 0 selects natural uint32_t wraparound. */
    float max_speed_m_s;
    float max_dt_s;
} encoder_speed_estimator_config_t;

typedef struct {
    encoder_speed_estimator_config_t config;
    uint32_t previous_count;
    bool has_previous_count;
    bool initialized;
} encoder_speed_estimator_t;

bool encoder_speed_estimator_init(
    encoder_speed_estimator_t *estimator,
    const encoder_speed_estimator_config_t *config);

/* Seed from the hardware timer after startup or after an encoder reset. */
bool encoder_speed_estimator_reset(
    encoder_speed_estimator_t *estimator,
    uint32_t counter_count);

/* Output is signed m/s; a bad interval/sample outputs zero and re-seeds safely. */
bool encoder_speed_estimator_update(
    encoder_speed_estimator_t *estimator,
    uint32_t counter_count,
    float dt_s,
    float *speed_m_s);

#endif
