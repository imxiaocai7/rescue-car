#include "rover_telemetry.h"

#include "motion_protocol.h"

#include <string.h>

static const uint8_t k_magic[4] = {'R', 'S', 'T', '1'};
static const uint8_t k_version = 1U;

static void write_le_u16(uint8_t *destination, uint16_t value) {
    destination[0] = (uint8_t)(value & 0xFFU);
    destination[1] = (uint8_t)((value >> 8U) & 0xFFU);
}

static void write_le_u32(uint8_t *destination, uint32_t value) {
    destination[0] = (uint8_t)(value & 0xFFUL);
    destination[1] = (uint8_t)((value >> 8U) & 0xFFUL);
    destination[2] = (uint8_t)((value >> 16U) & 0xFFUL);
    destination[3] = (uint8_t)((value >> 24U) & 0xFFUL);
}

static bool telemetry_is_valid(const rover_telemetry_t *telemetry) {
    const uint16_t flags = telemetry->status_flags;
    const bool left_tof_valid = (flags & ROVER_TELEMETRY_TOF_LEFT_VALID) != 0U;
    const bool right_tof_valid = (flags & ROVER_TELEMETRY_TOF_RIGHT_VALID) != 0U;
    const bool battery_valid = (flags & ROVER_TELEMETRY_BATTERY_VALID) != 0U;
    const bool gyro_valid = (flags & ROVER_TELEMETRY_GYRO_VALID) != 0U;
    const bool servo_open = (flags & ROVER_TELEMETRY_SERVO_OPEN) != 0U;
    const bool servo_closed = (flags & ROVER_TELEMETRY_SERVO_CLOSED) != 0U;

    if ((flags & (uint16_t)~ROVER_TELEMETRY_KNOWN_STATUS_MASK) != 0U
        || (servo_open && servo_closed)) {
        return false;
    }
    if (left_tof_valid) {
        if (telemetry->tof_left_mm < 20U || telemetry->tof_left_mm > 4000U) {
            return false;
        }
    } else if (telemetry->tof_left_mm != ROVER_TELEMETRY_INVALID_RANGE_MM) {
        return false;
    }
    if (right_tof_valid) {
        if (telemetry->tof_right_mm < 20U || telemetry->tof_right_mm > 4000U) {
            return false;
        }
    } else if (telemetry->tof_right_mm != ROVER_TELEMETRY_INVALID_RANGE_MM) {
        return false;
    }
    if ((battery_valid && telemetry->battery_mv == 0U)
        || (!battery_valid && telemetry->battery_mv != 0U)
        || (!gyro_valid && telemetry->gyro_z_mrad_s != 0)) {
        return false;
    }
    return true;
}

bool rover_telemetry_encode(
    const rover_telemetry_t *telemetry,
    uint8_t *frame,
    size_t frame_capacity) {
    uint16_t crc;
    if (telemetry == NULL || frame == NULL
        || frame_capacity < ROVER_TELEMETRY_FRAME_SIZE
        || !telemetry_is_valid(telemetry)) {
        return false;
    }

    memcpy(frame, k_magic, sizeof(k_magic));
    frame[4] = k_version;
    frame[5] = ROVER_TELEMETRY_TYPE;
    write_le_u32(frame + 6U, telemetry->sequence);
    write_le_u32(frame + 10U, telemetry->stm32_ticks_ms);
    write_le_u32(frame + 14U, telemetry->last_command_sequence);
    write_le_u32(frame + 18U, (uint32_t)telemetry->left_encoder_total);
    write_le_u32(frame + 22U, (uint32_t)telemetry->right_encoder_total);
    write_le_u16(frame + 26U, (uint16_t)telemetry->gyro_z_mrad_s);
    write_le_u16(frame + 28U, telemetry->tof_left_mm);
    write_le_u16(frame + 30U, telemetry->tof_right_mm);
    write_le_u16(frame + 32U, telemetry->battery_mv);
    write_le_u16(frame + 34U, telemetry->status_flags);
    crc = motion_crc16_ccitt_false(frame, ROVER_TELEMETRY_PREFIX_SIZE);
    write_le_u16(frame + ROVER_TELEMETRY_PREFIX_SIZE, crc);
    return true;
}
