#ifndef RESCUE_ROVER_APPLICATION_H
#define RESCUE_ROVER_APPLICATION_H

#include "encoder_speed_estimator.h"
#include "motion_control.h"
#include "motion_protocol.h"
#include "rover_telemetry.h"

typedef enum {
    ROVER_SERVO_STATE_UNKNOWN = 0U,
    ROVER_SERVO_STATE_OPEN = 1U,
    ROVER_SERVO_STATE_CLOSED = 2U
} rover_servo_state_t;

typedef struct {
    uint32_t now_ms;
    float dt_s;
    int32_t left_encoder_total;
    int32_t right_encoder_total;
    float left_measured_m_s;
    float right_measured_m_s;
    bool left_encoder_valid;
    bool right_encoder_valid;
    bool gyro_valid;
    int16_t gyro_z_mrad_s;
    bool tof_left_valid;
    uint16_t tof_left_mm;
    bool tof_right_valid;
    uint16_t tof_right_mm;
    bool battery_valid;
    uint16_t battery_mv;
    rover_servo_state_t servo_state;
    bool emergency_stop_active;
} rover_application_sample_t;

typedef struct {
    float left_duty_permille;
    float right_duty_permille;
    motion_actuator_command_t actuator;
    bool control_active;
    bool telemetry_valid;
    uint8_t telemetry_frame[ROVER_TELEMETRY_FRAME_SIZE];
} rover_application_output_t;

typedef struct {
    motion_decoder_t motion_decoder;
    motion_control_t motion_control;
    uint32_t telemetry_sequence;
    uint32_t last_command_sequence;
    bool has_accepted_command;
    bool initialized;
} rover_application_t;

bool rover_application_init(
    rover_application_t *application,
    const wheel_speed_controller_config_t *left_controller,
    const wheel_speed_controller_config_t *right_controller);

/* Call from the main application task, not directly from a UART ISR. */
bool rover_application_receive_uart(
    rover_application_t *application,
    const uint8_t *bytes,
    size_t length,
    uint32_t received_at_ms);

/* Call at a measured fixed-rate control interval after sampling hardware. */
bool rover_application_step(
    rover_application_t *application,
    const rover_application_sample_t *sample,
    rover_application_output_t *output);

#endif
