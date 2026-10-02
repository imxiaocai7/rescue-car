#include "commissioning_application.h"

#include <assert.h>
#include <stdio.h>
#include <string.h>
#ifdef _WIN32
#include <fcntl.h>
#include <io.h>
#endif

static uint32_t read_u32(const uint8_t *bytes) {
    return (uint32_t)bytes[0] | ((uint32_t)bytes[1] << 8U)
        | ((uint32_t)bytes[2] << 16U) | ((uint32_t)bytes[3] << 24U);
}

static void make_command(uint8_t *frame, uint32_t sequence, int16_t left, uint8_t actuator) {
    memset(frame, 0, MOTION_FRAME_SIZE);
    memcpy(frame, "RCM1", 4U);
    frame[4] = 1U;
    frame[5] = 1U;
    for (unsigned index = 0U; index < 4U; ++index) {
        frame[6U + index] = (uint8_t)(sequence >> (8U * index));
    }
    frame[10] = 100U;
    frame[12] = (uint8_t)(uint16_t)left;
    frame[13] = (uint8_t)((uint16_t)left >> 8U);
    frame[16] = actuator;
    const uint16_t crc = motion_crc16_ccitt_false(frame, MOTION_FRAME_SIZE - 2U);
    frame[18] = (uint8_t)crc;
    frame[19] = (uint8_t)(crc >> 8U);
}

static void test_interlocked_telemetry_and_zero_ack(void) {
    commissioning_application_t application;
    uint8_t frame[ROVER_TELEMETRY_FRAME_SIZE];
    uint8_t command[MOTION_FRAME_SIZE];
    commissioning_application_init(&application);
    assert(commissioning_application_step(&application, 0U, frame, sizeof(frame)));
    assert(memcmp(frame, "RST1", 4U) == 0);
    assert(frame[34] == 0U && frame[35] == 1U); /* Only software stop active. */
    assert(frame[28] == 0xFFU && frame[29] == 0xFFU);
    assert(frame[30] == 0xFFU && frame[31] == 0xFFU);
    assert(read_u32(frame + 18U) == 0U && read_u32(frame + 22U) == 0U);
    make_command(command, 42U, 0, 0U);
    commissioning_application_receive(&application, command, 7U, 10U);
    commissioning_application_receive(&application, command + 7U, MOTION_FRAME_SIZE - 7U, 12U);
    assert(application.accepted_zero_frames == 1U);
    assert(application.guard.received_at_ms == 12U);
    assert(commissioning_application_step(&application, 50U, frame, sizeof(frame)));
    assert(read_u32(frame + 14U) == 42U);
    assert(frame[34] == 0U && frame[35] == 3U); /* Stop + last command valid. */
    assert(application.guard.has_command);
    assert(commissioning_application_step(&application, 112U, frame, sizeof(frame)));
    assert(!application.guard.has_command);
    assert(application.guard.has_sequence && application.guard.last_sequence == 42U);
}

static void test_unsafe_commands_and_replay_are_never_accepted(void) {
    commissioning_application_t application;
    uint8_t frame[MOTION_FRAME_SIZE];
    commissioning_application_init(&application);
    make_command(frame, 10U, 0, 0U);
    commissioning_application_receive(&application, frame, sizeof(frame), 0U);
    commissioning_application_receive(&application, frame, sizeof(frame), 80U);
    assert(application.rejected_replay_frames == 1U);
    assert(application.guard.received_at_ms == 0U);
    make_command(frame, 11U, 1, 0U);
    commissioning_application_receive(&application, frame, sizeof(frame), 90U);
    make_command(frame, 12U, 0, 1U);
    commissioning_application_receive(&application, frame, sizeof(frame), 90U);
    make_command(frame, 13U, 0, 2U);
    commissioning_application_receive(&application, frame, sizeof(frame), 90U);
    assert(application.rejected_unsafe_frames == 3U);
    assert(application.guard.last_sequence == 10U);
}

static void test_stream_reset_discards_partial_frame_but_keeps_sequence(void) {
    commissioning_application_t application;
    uint8_t frame[MOTION_FRAME_SIZE];
    commissioning_application_init(&application);
    make_command(frame, 0xFFFFFFFFUL, 0, 0U);
    commissioning_application_receive(&application, frame, sizeof(frame), 0xFFFFFFF0UL);
    make_command(frame, 0U, 0, 0U);
    commissioning_application_receive(&application, frame, 9U, 0U);
    commissioning_application_stream_reset(&application);
    assert(!application.guard.has_command && application.guard.has_sequence);
    commissioning_application_receive(&application, frame + 9U, sizeof(frame) - 9U, 1U);
    assert(application.accepted_zero_frames == 1U);
    commissioning_application_receive(&application, frame, sizeof(frame), 2U);
    assert(application.accepted_zero_frames == 2U && application.guard.last_sequence == 0U);
    assert(application.stream_resets == 1U);
}

static void test_crc_recovery_and_invalid_arguments(void) {
    commissioning_application_t application;
    uint8_t command[MOTION_FRAME_SIZE];
    uint8_t telemetry[ROVER_TELEMETRY_FRAME_SIZE];
    commissioning_application_init(&application);
    make_command(command, 1U, 0, 0U);
    command[19] ^= 1U;
    commissioning_application_receive(&application, command, sizeof(command), 0U);
    assert(!application.guard.has_sequence && application.decoder.bad_checksums == 1U);
    make_command(command, 2U, 0, 0U);
    commissioning_application_receive(&application, command, sizeof(command), 1U);
    assert(application.guard.last_sequence == 2U);
    assert(!commissioning_application_step(NULL, 0U, telemetry, sizeof(telemetry)));
    assert(!commissioning_application_step(&application, 0U, telemetry, 1U));
    commissioning_application_init(NULL);
    commissioning_application_receive(NULL, NULL, 0U, 0U);
    commissioning_application_stream_reset(NULL);
}

int main(int argc, char **argv) {
    if (argc == 2 && strcmp(argv[1], "--wire") == 0) {
#ifdef _WIN32
        if (_setmode(_fileno(stdin), _O_BINARY) == -1) {
            return 1;
        }
#endif
        commissioning_application_t application;
        uint8_t frame[ROVER_TELEMETRY_FRAME_SIZE];
        commissioning_application_init(&application);
        int byte;
        while ((byte = getchar()) != EOF) {
            const uint8_t received = (uint8_t)byte;
            commissioning_application_receive(&application, &received, 1U, 10U);
        }
        if (!commissioning_application_step(&application, 42U, frame, sizeof(frame))) {
            return 1;
        }
        for (unsigned index = 0U; index < sizeof(frame); ++index) {
            printf("%02x", (unsigned)frame[index]);
        }
        return 0; /* Hex stdout avoids Windows text-mode newline translation. */
    }
    test_interlocked_telemetry_and_zero_ack();
    test_unsafe_commands_and_replay_are_never_accepted();
    test_stream_reset_discards_partial_frame_but_keeps_sequence();
    test_crc_recovery_and_invalid_arguments();
    puts("commissioning application tests passed (communication-only)");
    return 0;
}
