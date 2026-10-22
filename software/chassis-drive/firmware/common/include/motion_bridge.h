#ifndef CHASSIS_MOTION_BRIDGE_H
#define CHASSIS_MOTION_BRIDGE_H
#include "motion_protocol.h"
#ifdef __cplusplus
extern "C" {
#endif

#define CM_BRIDGE_REPLY_TIMEOUT_MS 150u

/* Policy used by ESP32, separate from the STM32 motor controller.
   No API generates an ACK: finish() accepts only a matching STM32 reply. */
typedef struct {
    bool pending, established, armed, have_ticks;
    uint32_t session, token, sequence, sent_ms, last_ticks;
    cm_request_t request;
} cm_bridge_t;

void cm_bridge_reset(cm_bridge_t *);
bool cm_bridge_expired(const cm_bridge_t *, uint32_t now);
bool cm_bridge_begin(cm_bridge_t *, const cm_request_t *, uint32_t now);
bool cm_bridge_finish(cm_bridge_t *, const cm_reply_t *, uint32_t now);

#ifdef __cplusplus
}
#endif
#endif
