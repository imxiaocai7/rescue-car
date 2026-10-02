#include "rover_hardware_profile.h"
#include "rover_telemetry.h"

#include <assert.h>
#include <stdio.h>
#include <string.h>

static rover_telemetry_t empty_telemetry(void) {
    rover_telemetry_t telemetry;
    memset(&telemetry, 0, sizeof(telemetry));
    telemetry.tof_left_mm = ROVER_TELEMETRY_INVALID_RANGE_MM;
    telemetry.tof_right_mm = ROVER_TELEMETRY_INVALID_RANGE_MM;
    return telemetry;
}

static void test_confirmed_encoder_profile(void) {
    assert(ROVER_MG513_HALL_PULSES_PER_MOTOR_REVOLUTION == 13U);
    assert(ROVER_MG513_GEAR_RATIO == 30U);
    assert(ROVER_MG513_COUNTS_PER_WHEEL_REVOLUTION == 1560U);
    assert(ROVER_MG513_STALL_CURRENT_MA == 3200U);
}

static void test_tinyf_range_boundaries(void) {
    uint8_t frame[ROVER_TELEMETRY_FRAME_SIZE];
    rover_telemetry_t telemetry = empty_telemetry();

    telemetry.status_flags = ROVER_TELEMETRY_TOF_LEFT_VALID;
    telemetry.tof_left_mm = ROVER_TINYF_MIN_RANGE_MM;
    assert(rover_telemetry_encode(&telemetry, frame, sizeof(frame)));

    telemetry.tof_left_mm = ROVER_TINYF_MAX_RANGE_MM;
    assert(rover_telemetry_encode(&telemetry, frame, sizeof(frame)));

    telemetry.tof_left_mm = ROVER_TINYF_MIN_RANGE_MM - 1U;
    assert(!rover_telemetry_encode(&telemetry, frame, sizeof(frame)));

    telemetry.tof_left_mm = ROVER_TINYF_MAX_RANGE_MM + 1U;
    assert(!rover_telemetry_encode(&telemetry, frame, sizeof(frame)));
}

int main(void) {
    test_confirmed_encoder_profile();
    test_tinyf_range_boundaries();
    puts("rover_hardware_profile_test: all tests passed");
    return 0;
}
