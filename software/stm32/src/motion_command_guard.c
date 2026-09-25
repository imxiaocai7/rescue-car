#include "motion_command_guard.h"

#include <string.h>

void motion_command_guard_init(motion_command_guard_t *guard) {
    if (guard == NULL) {
        return;
    }
    memset(guard, 0, sizeof(*guard));
}

bool motion_command_guard_accept(
    motion_command_guard_t *guard,
    const motion_command_t *command,
    uint32_t received_at_ms) {
    if (guard == NULL || command == NULL) {
        return false;
    }
    if (command->lease_ms == 0U
        || command->lease_ms > MOTION_MAX_LEASE_MS
        || command->actuator > MOTION_ACTUATOR_OPEN) {
        return false;
    }
    if (guard->has_command
        && !motion_sequence_is_newer(command->sequence, guard->active_command.sequence)) {
        return false;
    }

    guard->active_command = *command;
    guard->received_at_ms = received_at_ms;
    guard->has_command = true;
    return true;
}

bool motion_command_guard_get(
    const motion_command_guard_t *guard,
    uint32_t now_ms,
    motion_command_t *output) {
    if (output == NULL) {
        return false;
    }
    if (guard == NULL
        || !guard->has_command
        || motion_lease_expired(now_ms, guard->received_at_ms, guard->active_command.lease_ms)) {
        memset(output, 0, sizeof(*output));
        output->actuator = MOTION_ACTUATOR_HOLD;
        return false;
    }
    *output = guard->active_command;
    return true;
}
