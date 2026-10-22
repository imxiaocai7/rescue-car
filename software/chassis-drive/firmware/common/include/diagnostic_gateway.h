#ifndef CHASSIS_DIAGNOSTIC_GATEWAY_H
#define CHASSIS_DIAGNOSTIC_GATEWAY_H
#include "diagnostic_protocol.h"

#ifdef __cplusplus
extern "C" {
#endif

#define CD_GATEWAY_TIMEOUT_MS 400u
typedef struct { bool pending; uint32_t sequence; uint32_t sent_ms; } cd_gateway_t;
void cd_gateway_reset(cd_gateway_t *gateway);
bool cd_gateway_expired(const cd_gateway_t *gateway, uint32_t now_ms);
bool cd_gateway_begin(cd_gateway_t *gateway, uint32_t sequence, uint32_t now_ms);
bool cd_gateway_finish(cd_gateway_t *gateway, const cd_reply_t *reply, uint32_t now_ms);

#ifdef __cplusplus
}
#endif
#endif
