#include "rover_application.h"

#include "rover_hardware_profile.h"

#include <math.h>
#include <stddef.h>
#include <string.h>

typedef struct {
    rover_application_t *application;
    uint32_t received_at_ms;
    bool accepted_any;
} command_context_t;

static void handle_motion_command(const motion_command_t *command, void *context) {
    command_context_t *command_context = (command_context_t *)context;
    if (command_context == NULL || command_context->application == NULL) {
        return;
    }
    rover_application_t *application = command_context->application;
    if (motion_control_accept(
            &application->motion_control,
            command,
            command_context->received_at_ms)) {
        application->last_command_sequence = command->sequence;
        application->has_accepted_command = true;
        command_context->accepted_any = true;
    }
}

static void stop_output(
    rover_application_t *application,
    rover_application_output_t *output) {
    if (application != NULL && application->initialized) {
        motion_control_disarm(&application->motion_control);
    }
    output->left_duty_permille = 0.0F;
    output->right_duty_permille = 0.0F;
    output->actuator = MOTION_ACTUATOR_HOLD;
    output->control_active = false;
}

bool rover_application_init(
    rover_application_t *application,
    const wheel_speed_controller_config_t *left_controller,
    const wheel_speed_controller_config_t *right_controller) {
    if (application == NULL) {
        return false;
    }
    memset(application, 0, sizeof(*application));
    motion_decoder_init(&application->motion_decoder);
    if (!motion_control_init(
            &application->motion_control,
            left_controller,
            right_controller)) {
        return false;
    }
    application->initialized = true;
    return true;
}

bool rover_application_receive_uart(
    rover_application_t *application,
    const uint8_t *bytes,
    size_t length,
    uint32_t received_at_ms) {
    if (application == NULL || !application->initialized
        || (bytes == NULL && length != 0U)) {
        return false;
    }
    command_context_t context;
    context.application = application;
    context.received_at_ms = received_at_ms;
    context.accepted_any = false;
    motion_decoder_feed(
        &application->motion_decoder,
        bytes,
        length,
        handle_motion_command,
        &context);
    return context.accepted_any;
}

bool rover_application_step(
    rover_application_t *application,
    const rover_application_sample_t *sample,
    rover_application_output_t *output) {
    if (output == NULL) {
        return false;
    }
    memset(output, 0, sizeof(*output));
    output->actuator = MOTION_ACTUATOR_HOLD;
    if (application == NULL || !application->initialized || sample == NULL) {
        return false;
    }

    const bool encoders_valid = sample->left_encoder_valid
        && sample->right_encoder_valid;
    const bool timing_valid = isfinite(sample->dt_s)
        && sample->dt_s > 0.0F;
    bool control_active = false;
    if (encoders_valid && timing_valid && !sample->emergency_stop_active) {
        motion_control_output_t control_output;
        control_active = motion_control_update(
            &application->motion_control,
            sample->now_ms,
            sample->left_measured_m_s,
            sample->right_measured_m_s,
            sample->dt_s,
            &control_output);
        output->left_duty_permille = control_output.left_duty_permille;
        output->right_duty_permille = control_output.right_duty_permille;
        output->actuator = control_output.actuator;
    }
    if (!control_active) {
        stop_output(application, output);
    } else {
        output->control_active = true;
    }

    rover_telemetry_t telemetry;
    memset(&telemetry, 0, sizeof(telemetry));
    telemetry.sequence = application->telemetry_sequence++;
    telemetry.stm32_ticks_ms = sample->now_ms;
    telemetry.last_command_sequence = application->last_command_sequence;
    telemetry.left_encoder_total = sample->left_encoder_total;
    telemetry.right_encoder_total = sample->right_encoder_total;
    telemetry.gyro_z_mrad_s = sample->gyro_valid ? sample->gyro_z_mrad_s : 0;
    telemetry.tof_left_mm = ROVER_TELEMETRY_INVALID_RANGE_MM;
    telemetry.tof_right_mm = ROVER_TELEMETRY_INVALID_RANGE_MM;
    telemetry.battery_mv = sample->battery_valid ? sample->battery_mv : 0U;

    if (sample->left_encoder_valid) {
        telemetry.status_flags |= ROVER_TELEMETRY_LEFT_ENCODER_VALID;
    }
    if (sample->right_encoder_valid) {
        telemetry.status_flags |= ROVER_TELEMETRY_RIGHT_ENCODER_VALID;
    }
    if (sample->gyro_valid) {
        telemetry.status_flags |= ROVER_TELEMETRY_GYRO_VALID;
    }
    if (sample->tof_left_valid
        && sample->tof_left_mm >= ROVER_TINYF_MIN_RANGE_MM
        && sample->tof_left_mm <= ROVER_TINYF_MAX_RANGE_MM) {
        telemetry.tof_left_mm = sample->tof_left_mm;
        telemetry.status_flags |= ROVER_TELEMETRY_TOF_LEFT_VALID;
    }
    if (sample->tof_right_valid
        && sample->tof_right_mm >= ROVER_TINYF_MIN_RANGE_MM
        && sample->tof_right_mm <= ROVER_TINYF_MAX_RANGE_MM) {
        telemetry.tof_right_mm = sample->tof_right_mm;
        telemetry.status_flags |= ROVER_TELEMETRY_TOF_RIGHT_VALID;
    }
    if (sample->battery_valid && sample->battery_mv > 0U) {
        telemetry.status_flags |= ROVER_TELEMETRY_BATTERY_VALID;
    }
    if (sample->servo_state == ROVER_SERVO_STATE_OPEN) {
        telemetry.status_flags |= ROVER_TELEMETRY_SERVO_OPEN;
    } else if (sample->servo_state == ROVER_SERVO_STATE_CLOSED) {
        telemetry.status_flags |= ROVER_TELEMETRY_SERVO_CLOSED;
    }
    if (sample->emergency_stop_active) {
        telemetry.status_flags |= ROVER_TELEMETRY_EMERGENCY_STOP;
    }
    if (application->has_accepted_command) {
        telemetry.status_flags |= ROVER_TELEMETRY_LAST_COMMAND_VALID;
    }

    output->telemetry_valid = rover_telemetry_encode(
        &telemetry,
        output->telemetry_frame,
        sizeof(output->telemetry_frame));
    if (!output->telemetry_valid) {
        stop_output(application, output);
    }
    return output->telemetry_valid;
}
