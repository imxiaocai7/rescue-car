#ifndef RESCUE_MOTION_PROTOCOL_H
#define RESCUE_MOTION_PROTOCOL_H

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#define MOTION_FRAME_SIZE 20U
#define MOTION_MAX_LEASE_MS 1000U

typedef enum {
    MOTION_ACTUATOR_HOLD = 0U,
    MOTION_ACTUATOR_CLOSE = 1U,
    MOTION_ACTUATOR_OPEN = 2U
} motion_actuator_command_t;

typedef struct {
    uint32_t sequence;
    uint16_t lease_ms;
    int16_t left_mm_s;
    int16_t right_mm_s;
    motion_actuator_command_t actuator;
} motion_command_t;

typedef struct {
    uint8_t buffer[MOTION_FRAME_SIZE];
    size_t length;
    uint32_t frames_decoded;
    uint32_t bad_headers;
    uint32_t bad_checksums;
} motion_decoder_t;

typedef void (*motion_command_handler_t)(const motion_command_t *command, void *context);

void motion_decoder_init(motion_decoder_t *decoder);
void motion_decoder_feed(
    motion_decoder_t *decoder,
    const uint8_t *data,
    size_t length,
    motion_command_handler_t handler,
    void *context);

uint16_t motion_crc16_ccitt_false(const uint8_t *data, size_t length);
bool motion_sequence_is_newer(uint32_t candidate, uint32_t previous);
bool motion_lease_expired(uint32_t now_ms, uint32_t received_ms, uint16_t lease_ms);

#endif
