#include "diagnostic_gateway.h"

void cd_gateway_reset(cd_gateway_t *gateway)
{
    gateway->pending = false;
    gateway->sequence = 0;
    gateway->sent_ms = 0;
}

bool cd_gateway_expired(const cd_gateway_t *gateway, uint32_t now_ms)
{
    return gateway->pending && (uint32_t)(now_ms - gateway->sent_ms) >= CD_GATEWAY_TIMEOUT_MS;
}

bool cd_gateway_begin(cd_gateway_t *gateway, uint32_t sequence, uint32_t now_ms)
{
    if (gateway->pending) return false;
    gateway->pending = true;
    gateway->sequence = sequence;
    gateway->sent_ms = now_ms;
    return true;
}

bool cd_gateway_finish(cd_gateway_t *gateway, const cd_reply_t *reply, uint32_t now_ms)
{
    if (!gateway->pending || cd_gateway_expired(gateway, now_ms) ||
        reply->sequence != gateway->sequence) return false;
    if (!((reply->firmware_id==CD_FIRMWARE_ID && reply->flags==CD_NO_MOTOR_IMPLEMENTATION) ||
          (reply->firmware_id==CD_MOTOR_FIRMWARE_ID && reply->flags==CD_MOTOR_IMPLEMENTED))) return false;
    gateway->pending = false;
    return true;
}
