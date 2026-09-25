#ifndef RESCUE_MOTION_COMMAND_GUARD_H
#define RESCUE_MOTION_COMMAND_GUARD_H

#include "motion_protocol.h"

typedef struct {
    bool has_command;
    uint32_t received_at_ms;
    motion_command_t active_command;
} motion_command_guard_t;

void motion_command_guard_init(motion_command_guard_t *guard);
bool motion_command_guard_accept(
    motion_command_guard_t *guard,
    const motion_command_t *command,
    uint32_t received_at_ms);

/* Returns true only while a fresh command is leased. Always writes an output. */
bool motion_command_guard_get(
    const motion_command_guard_t *guard,
    uint32_t now_ms,
    motion_command_t *output);

#endif
