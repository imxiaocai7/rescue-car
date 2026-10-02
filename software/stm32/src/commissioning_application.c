#include "commissioning_application.h"

#include <string.h>

typedef struct {
    commissioning_application_t *application;
    uint32_t received_at_ms;
} commissioning_context_t;

static void receive_command(const motion_command_t *command, void *context) {
    commissioning_context_t *receive_context = (commissioning_context_t *)context;
    commissioning_application_t *application = receive_context->application;
    if (command->left_mm_s != 0 || command->right_mm_s != 0
        || command->actuator != MOTION_ACTUATOR_HOLD) {
        ++application->rejected_unsafe_frames;
        return;
    }
    if (motion_command_guard_accept(
            &application->guard, command, receive_context->received_at_ms)) {
        ++application->accepted_zero_frames;
    } else {
        ++application->rejected_replay_frames;
    }
}

void commissioning_application_init(commissioning_application_t *application) {
    if (application != NULL) {
        memset(application, 0, sizeof(*application));
        motion_decoder_init(&application->decoder);
        motion_command_guard_init(&application->guard);
    }
}

void commissioning_application_receive(
    commissioning_application_t *application,
    const uint8_t *bytes, size_t length, uint32_t received_at_ms) {
    if (application == NULL || (bytes == NULL && length != 0U)) {
        return;
    }
    commissioning_context_t context = {application, received_at_ms};
    motion_decoder_feed(&application->decoder, bytes, length, receive_command, &context);
}

void commissioning_application_stream_reset(commissioning_application_t *application) {
    if (application != NULL) {
        application->decoder.length = 0U;
        motion_command_guard_disarm(&application->guard);
        ++application->stream_resets;
    }
}

bool commissioning_application_step(
    commissioning_application_t *application, uint32_t now_ms,
    uint8_t *telemetry_frame, size_t frame_capacity) {
    if (application == NULL) {
        return false;
    }
    motion_command_t unused_zero;
    if (!motion_command_guard_get(&application->guard, now_ms, &unused_zero)) {
        motion_command_guard_disarm(&application->guard);
    }
    rover_telemetry_t telemetry;
    memset(&telemetry, 0, sizeof(telemetry));
    telemetry.sequence = application->telemetry_sequence;
    telemetry.stm32_ticks_ms = now_ms;
    telemetry.last_command_sequence = application->guard.last_sequence;
    telemetry.tof_left_mm = ROVER_TELEMETRY_INVALID_RANGE_MM;
    telemetry.tof_right_mm = ROVER_TELEMETRY_INVALID_RANGE_MM;
    /* EMERGENCY_STOP denotes our permanent software commissioning interlock,
       not a claim that a physical emergency button has been wired or tested. */
    telemetry.status_flags = ROVER_TELEMETRY_EMERGENCY_STOP;
    if (application->guard.has_sequence) {
        telemetry.status_flags |= ROVER_TELEMETRY_LAST_COMMAND_VALID;
    }
    if (!rover_telemetry_encode(&telemetry, telemetry_frame, frame_capacity)) {
        return false;
    }
    ++application->telemetry_sequence;
    return true;
}
