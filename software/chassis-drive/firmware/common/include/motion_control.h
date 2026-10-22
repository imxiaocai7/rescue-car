#ifndef CHASSIS_MOTION_CONTROL_H
#define CHASSIS_MOTION_CONTROL_H
#include "motion_protocol.h"
#ifdef __cplusplus
extern "C" {
#endif
typedef struct {
    uint32_t session, token, challenge, sequence, last_command_ms, ramp_ms, flags;
    int16_t target[2], output[2];
    uint32_t reverse_until[2];
    bool reversing[2];
} cm_control_t;
void cm_control_init(cm_control_t *, uint32_t boot_random, uint32_t now);
void cm_control_disable(cm_control_t *, uint32_t reason);
void cm_control_tick(cm_control_t *, uint32_t now);
cm_result_t cm_control_request(cm_control_t *, const cm_request_t *, uint32_t now);
void cm_control_reply(const cm_control_t *, const cm_request_t *, uint32_t now,
                      cm_result_t, cm_reply_t *);
#ifdef __cplusplus
}
#endif
#endif
