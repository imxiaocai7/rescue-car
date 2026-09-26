#ifndef RESCUE_ROVER_TELEMETRY_H
#define RESCUE_ROVER_TELEMETRY_H

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#define ROVER_TELEMETRY_PREFIX_SIZE 36U
#define ROVER_TELEMETRY_FRAME_SIZE 38U
#define ROVER_TELEMETRY_TYPE 2U
#define ROVER_TELEMETRY_INVALID_RANGE_MM 0xFFFFU

#define ROVER_TELEMETRY_LEFT_ENCODER_VALID (1U << 0U)
#define ROVER_TELEMETRY_RIGHT_ENCODER_VALID (1U << 1U)
#define ROVER_TELEMETRY_GYRO_VALID (1U << 2U)
#define ROVER_TELEMETRY_TOF_LEFT_VALID (1U << 3U)
#define ROVER_TELEMETRY_TOF_RIGHT_VALID (1U << 4U)
#define ROVER_TELEMETRY_BATTERY_VALID (1U << 5U)
#define ROVER_TELEMETRY_SERVO_OPEN (1U << 6U)
#define ROVER_TELEMETRY_SERVO_CLOSED (1U << 7U)
#define ROVER_TELEMETRY_EMERGENCY_STOP (1U << 8U)
#define ROVER_TELEMETRY_LAST_COMMAND_VALID (1U << 9U)
#define ROVER_TELEMETRY_KNOWN_STATUS_MASK 0x03FFU

typedef struct {
    uint32_t sequence;
    uint32_t stm32_ticks_ms;
    uint32_t last_command_sequence;
    int32_t left_encoder_total;
    int32_t right_encoder_total;
    int16_t gyro_z_mrad_s;
    uint16_t tof_left_mm;
    uint16_t tof_right_mm;
    uint16_t battery_mv;
    uint16_t status_flags;
} rover_telemetry_t;

/* Write one validated RST1 telemetry frame. Requires motion_protocol.c for CRC. */
bool rover_telemetry_encode(
    const rover_telemetry_t *telemetry,
    uint8_t *frame,
    size_t frame_capacity);

#endif
