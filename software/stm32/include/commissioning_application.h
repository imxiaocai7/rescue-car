#ifndef RESCUE_COMMISSIONING_APPLICATION_H
#define RESCUE_COMMISSIONING_APPLICATION_H

#include "motion_command_guard.h"
#include "rover_telemetry.h"

/* Phase 2 has no motor/servo output API and never reports fake sensor validity. */
typedef struct {
    motion_decoder_t decoder;
    motion_command_guard_t guard;
    uint32_t telemetry_sequence;
    uint32_t accepted_zero_frames;
    uint32_t rejected_unsafe_frames;
    uint32_t rejected_replay_frames;
    uint32_t stream_resets;
} commissioning_application_t;

void commissioning_application_init(commissioning_application_t *application);
void commissioning_application_receive(
    commissioning_application_t *application,
    const uint8_t *bytes, size_t length, uint32_t received_at_ms);
/* UART errors/queue overflow clear partial frames and leases, not sequence history. */
void commissioning_application_stream_reset(commissioning_application_t *application);
bool commissioning_application_step(
    commissioning_application_t *application, uint32_t now_ms,
    uint8_t *telemetry_frame, size_t frame_capacity);

#endif
