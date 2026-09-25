#include "motion_protocol.h"

#include <string.h>

static const uint8_t k_magic[4] = {'R', 'C', 'M', '1'};
static const uint8_t k_version = 1U;
static const uint8_t k_command_type = 1U;

static uint16_t read_le_u16(const uint8_t *data) {
    return (uint16_t)((uint16_t)data[0] | ((uint16_t)data[1] << 8U));
}

static uint32_t read_le_u32(const uint8_t *data) {
    return (uint32_t)data[0]
        | ((uint32_t)data[1] << 8U)
        | ((uint32_t)data[2] << 16U)
        | ((uint32_t)data[3] << 24U);
}

static int16_t read_le_i16(const uint8_t *data) {
    const uint16_t raw = read_le_u16(data);
    const int32_t signed_value = (raw <= INT16_MAX)
        ? (int32_t)raw
        : (int32_t)raw - 65536L;
    return (int16_t)signed_value;
}

static void discard_prefix(motion_decoder_t *decoder, size_t count) {
    if (count >= decoder->length) {
        decoder->length = 0U;
        return;
    }
    decoder->length -= count;
    memmove(decoder->buffer, decoder->buffer + count, decoder->length);
}

static size_t suffix_matching_magic_prefix(const uint8_t *data, size_t length) {
    size_t keep = length < 3U ? length : 3U;
    while (keep > 0U) {
        if (memcmp(data + length - keep, k_magic, keep) == 0) {
            return keep;
        }
        --keep;
    }
    return 0U;
}

static void process_buffer(
    motion_decoder_t *decoder,
    motion_command_handler_t handler,
    void *context) {
    for (;;) {
        size_t start = 0U;
        bool found_magic = false;

        while (start + sizeof(k_magic) <= decoder->length) {
            if (memcmp(decoder->buffer + start, k_magic, sizeof(k_magic)) == 0) {
                found_magic = true;
                break;
            }
            ++start;
        }

        if (!found_magic) {
            const size_t keep = suffix_matching_magic_prefix(decoder->buffer, decoder->length);
            discard_prefix(decoder, decoder->length - keep);
            return;
        }
        if (start > 0U) {
            discard_prefix(decoder, start);
        }
        if (decoder->length < MOTION_FRAME_SIZE) {
            return;
        }

        const uint8_t *frame = decoder->buffer;
        const uint16_t lease_ms = read_le_u16(frame + 10U);
        const uint8_t actuator = frame[16U];
        if (frame[4U] != k_version
            || frame[5U] != k_command_type
            || lease_ms == 0U
            || lease_ms > MOTION_MAX_LEASE_MS
            || actuator > MOTION_ACTUATOR_OPEN
            || frame[17U] != 0U) {
            ++decoder->bad_headers;
            discard_prefix(decoder, 1U);
            continue;
        }

        const uint16_t expected_crc = read_le_u16(frame + 18U);
        if (motion_crc16_ccitt_false(frame, 18U) != expected_crc) {
            ++decoder->bad_checksums;
            discard_prefix(decoder, 1U);
            continue;
        }

        motion_command_t command;
        command.sequence = read_le_u32(frame + 6U);
        command.lease_ms = lease_ms;
        command.left_mm_s = read_le_i16(frame + 12U);
        command.right_mm_s = read_le_i16(frame + 14U);
        command.actuator = (motion_actuator_command_t)actuator;
        ++decoder->frames_decoded;
        if (handler != NULL) {
            handler(&command, context);
        }
        discard_prefix(decoder, MOTION_FRAME_SIZE);
    }
}

void motion_decoder_init(motion_decoder_t *decoder) {
    if (decoder == NULL) {
        return;
    }
    memset(decoder, 0, sizeof(*decoder));
}

void motion_decoder_feed(
    motion_decoder_t *decoder,
    const uint8_t *data,
    size_t length,
    motion_command_handler_t handler,
    void *context) {
    size_t index;
    if (decoder == NULL || (data == NULL && length != 0U)) {
        return;
    }
    for (index = 0U; index < length; ++index) {
        if (decoder->length >= MOTION_FRAME_SIZE) {
            ++decoder->bad_headers;
            discard_prefix(decoder, 1U);
        }
        decoder->buffer[decoder->length] = data[index];
        ++decoder->length;
        process_buffer(decoder, handler, context);
    }
}

uint16_t motion_crc16_ccitt_false(const uint8_t *data, size_t length) {
    uint16_t crc = 0xFFFFU;
    size_t index;
    if (data == NULL && length != 0U) {
        return 0U;
    }
    for (index = 0U; index < length; ++index) {
        unsigned bit;
        crc ^= (uint16_t)((uint16_t)data[index] << 8U);
        for (bit = 0U; bit < 8U; ++bit) {
            if ((crc & 0x8000U) != 0U) {
                crc = (uint16_t)((crc << 1U) ^ 0x1021U);
            } else {
                crc = (uint16_t)(crc << 1U);
            }
        }
    }
    return crc;
}

bool motion_sequence_is_newer(uint32_t candidate, uint32_t previous) {
    const uint32_t delta = candidate - previous;
    return delta != 0U && delta < 0x80000000UL;
}

bool motion_lease_expired(uint32_t now_ms, uint32_t received_ms, uint16_t lease_ms) {
    if (lease_ms == 0U || lease_ms > MOTION_MAX_LEASE_MS) {
        return true;
    }
    return (uint32_t)(now_ms - received_ms) >= lease_ms;
}
