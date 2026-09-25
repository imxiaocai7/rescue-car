#include "encoder_speed_estimator.h"

#include <assert.h>
#include <math.h>
#include <stdio.h>

static encoder_speed_estimator_config_t test_config(void) {
    encoder_speed_estimator_config_t config = {
        0.0375F,
        1000U,
        65536U,
        1.0F,
        0.1F
    };
    return config;
}

static float expected_speed(int32_t delta_counts, float dt_s) {
    const float circumference = 6.2831853071795864769F * 0.0375F;
    return ((float)delta_counts * circumference / 1000.0F) / dt_s;
}

static void test_signed_speed_forward_reverse_and_wrap(void) {
    encoder_speed_estimator_t estimator;
    encoder_speed_estimator_config_t config = test_config();
    float speed = 0.0F;
    assert(encoder_speed_estimator_init(&estimator, &config));
    assert(encoder_speed_estimator_reset(&estimator, 1000U));
    assert(encoder_speed_estimator_update(&estimator, 1010U, 0.1F, &speed));
    assert(fabsf(speed - expected_speed(10, 0.1F)) < 0.00001F);
    assert(encoder_speed_estimator_update(&estimator, 1000U, 0.1F, &speed));
    assert(fabsf(speed - expected_speed(-10, 0.1F)) < 0.00001F);

    assert(encoder_speed_estimator_reset(&estimator, 65530U));
    assert(encoder_speed_estimator_update(&estimator, 5U, 0.1F, &speed));
    assert(fabsf(speed - expected_speed(11, 0.1F)) < 0.00001F);
}

static void test_uint32_natural_wrap(void) {
    encoder_speed_estimator_t estimator;
    encoder_speed_estimator_config_t config = test_config();
    float speed = 0.0F;
    config.counter_modulus = 0U;
    assert(encoder_speed_estimator_init(&estimator, &config));
    assert(encoder_speed_estimator_reset(&estimator, 0xFFFFFFFCUL));
    assert(encoder_speed_estimator_update(&estimator, 3U, 0.1F, &speed));
    assert(fabsf(speed - expected_speed(7, 0.1F)) < 0.00001F);
}

static void test_ambiguous_jump_invalid_interval_and_speed_fail_closed(void) {
    encoder_speed_estimator_t estimator;
    encoder_speed_estimator_config_t config = test_config();
    float speed = 123.0F;
    assert(encoder_speed_estimator_init(&estimator, &config));
    assert(encoder_speed_estimator_reset(&estimator, 0U));
    assert(!encoder_speed_estimator_update(&estimator, 32768U, 0.05F, &speed));
    assert(speed == 0.0F);

    assert(!encoder_speed_estimator_update(&estimator, 10U, 0.2F, &speed));
    assert(speed == 0.0F);
    /* The invalid interval re-seeded at 10; this next result uses only one tick. */
    assert(encoder_speed_estimator_update(&estimator, 11U, 0.1F, &speed));
    assert(fabsf(speed - expected_speed(1, 0.1F)) < 0.00001F);

    assert(!encoder_speed_estimator_update(&estimator, 1000U, 0.01F, &speed));
    assert(speed == 0.0F);
}

static void test_configuration_and_counter_range_validation(void) {
    encoder_speed_estimator_t estimator;
    encoder_speed_estimator_config_t config = test_config();
    config.counts_per_wheel_revolution = 0U;
    assert(!encoder_speed_estimator_init(&estimator, &config));

    config = test_config();
    assert(encoder_speed_estimator_init(&estimator, &config));
    assert(!encoder_speed_estimator_reset(&estimator, config.counter_modulus));
}

int main(void) {
    test_signed_speed_forward_reverse_and_wrap();
    test_uint32_natural_wrap();
    test_ambiguous_jump_invalid_interval_and_speed_fail_closed();
    test_configuration_and_counter_range_validation();
    puts("encoder_speed_estimator_test: all tests passed");
    return 0;
}
