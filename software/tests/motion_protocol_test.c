#include "motion_protocol.h"
#include "motion_command_guard.h"

#include <assert.h>
#include <stdio.h>
#include <string.h>

typedef struct {
    size_t count;
    motion_command_t last;
} command_log_t;

static void record_command(const motion_command_t *command, void *context) {
    command_log_t *log = (command_log_t *)context;
    ++log->count;
    log->last = *command;
}

static void test_crc_check_value(void) {
    static const uint8_t input[] = "123456789";
    assert(motion_crc16_ccitt_false(input, sizeof(input) - 1U) == 0x29B1U);
}

static void test_fragmented_command(void) {
    static const uint8_t frame[MOTION_FRAME_SIZE] = {
        0x52, 0x43, 0x4D, 0x31, 0x01, 0x01, 0x78, 0x56, 0x34, 0x12,
        0x64, 0x00, 0x85, 0xFF, 0xC8, 0x01, 0x02, 0x00, 0xCC, 0xC1
    };
    motion_decoder_t decoder;
    command_log_t log = {0};
    motion_decoder_init(&decoder);

    motion_decoder_feed(&decoder, frame, 3U, record_command, &log);
    motion_decoder_feed(&decoder, frame + 3U, 8U, record_command, &log);
    assert(log.count == 0U);
    motion_decoder_feed(&decoder, frame + 11U, MOTION_FRAME_SIZE - 11U, record_command, &log);

    assert(log.count == 1U);
    assert(log.last.sequence == 0x12345678UL);
    assert(log.last.lease_ms == 100U);
    assert(log.last.left_mm_s == -123);
    assert(log.last.right_mm_s == 456);
    assert(log.last.actuator == MOTION_ACTUATOR_OPEN);
    assert(decoder.frames_decoded == 1U);
}

static void test_noise_bad_crc_and_recovery(void) {
    static const uint8_t good[MOTION_FRAME_SIZE] = {
        0x52, 0x43, 0x4D, 0x31, 0x01, 0x01, 0x78, 0x56, 0x34, 0x12,
        0x64, 0x00, 0x85, 0xFF, 0xC8, 0x01, 0x02, 0x00, 0xCC, 0xC1
    };
    uint8_t stream[5U + MOTION_FRAME_SIZE * 2U];
    motion_decoder_t decoder;
    command_log_t log = {0};
    memset(stream, 0xA5, 5U);
    memcpy(stream + 5U, good, MOTION_FRAME_SIZE);
    memcpy(stream + 5U + MOTION_FRAME_SIZE, good, MOTION_FRAME_SIZE);
    stream[5U + 18U] ^= 0x01U;
    motion_decoder_init(&decoder);
    motion_decoder_feed(&decoder, stream, sizeof(stream), record_command, &log);

    assert(log.count == 1U);
    assert(decoder.bad_checksums == 1U);
    assert(decoder.frames_decoded == 1U);
}

static void test_sequence_and_watchdog_wrap(void) {
    assert(motion_sequence_is_newer(0U, 0xFFFFFFFFUL));
    assert(motion_sequence_is_newer(8U, 7U));
    assert(!motion_sequence_is_newer(7U, 7U));
    assert(!motion_sequence_is_newer(7U, 8U));
    assert(!motion_lease_expired(0xFFFFFFF9UL, 0xFFFFFFF0UL, 20U));
    assert(motion_lease_expired(4U, 0xFFFFFFF0UL, 20U));
    assert(motion_lease_expired(0U, 0U, 0U));
}

static void test_command_guard_rejects_replay_and_stops_on_expiry(void) {
    motion_command_guard_t guard;
    motion_command_t command = {
        0xFFFFFFFFUL, 20U, 120, 120, MOTION_ACTUATOR_OPEN
    };
    motion_command_t output;
    motion_command_guard_init(&guard);
    assert(!motion_command_guard_get(&guard, 0U, &output));
    assert(output.left_mm_s == 0 && output.right_mm_s == 0);

    assert(motion_command_guard_accept(&guard, &command, 0xFFFFFFF0UL));
    assert(motion_command_guard_get(&guard, 0xFFFFFFF9UL, &output));
    assert(output.left_mm_s == 120 && output.right_mm_s == 120);
    assert(!motion_command_guard_accept(&guard, &command, 0xFFFFFFFAUL));

    command.sequence = 0U;  /* Valid wrap after UINT32_MAX. */
    command.lease_ms = 10U;
    assert(motion_command_guard_accept(&guard, &command, 0xFFFFFFFAUL));
    assert(motion_command_guard_get(&guard, 3U, &output));
    assert(!motion_command_guard_get(&guard, 4U, &output));
    assert(output.sequence == 0U);
    assert(output.lease_ms == 0U);
    assert(output.left_mm_s == 0 && output.right_mm_s == 0);
    assert(output.actuator == MOTION_ACTUATOR_HOLD);
}

int main(void) {
    test_crc_check_value();
    test_fragmented_command();
    test_noise_bad_crc_and_recovery();
    test_sequence_and_watchdog_wrap();
    test_command_guard_rejects_replay_and_stops_on_expiry();
    puts("motion_protocol_test: all tests passed");
    return 0;
}
