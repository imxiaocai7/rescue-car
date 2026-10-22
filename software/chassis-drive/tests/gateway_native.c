#include "diagnostic_gateway.h"
#include <assert.h>
#include <stdio.h>

int main(void)
{
    cd_gateway_t gateway;
    cd_gateway_reset(&gateway);
    cd_reply_t reply = {42, 100, CD_FIRMWARE_ID, CD_NO_MOTOR_IMPLEMENTATION};
    assert(!cd_gateway_finish(&gateway, &reply, 0));
    assert(cd_gateway_begin(&gateway, 42, 0));
    assert(!cd_gateway_begin(&gateway, 43, 1));
    assert(!cd_gateway_expired(&gateway, 399));
    assert(cd_gateway_finish(&gateway, &reply, 399));
    assert(!cd_gateway_finish(&gateway, &reply, 399));
    assert(cd_gateway_begin(&gateway, 42, 0));
    assert(cd_gateway_expired(&gateway, 400));
    assert(!cd_gateway_finish(&gateway, &reply, 400));
    cd_gateway_reset(&gateway);
    assert(cd_gateway_begin(&gateway, 42, UINT32_C(0xFFFFFFF0)));
    assert(cd_gateway_finish(&gateway, &reply, 16));
    assert(cd_gateway_begin(&gateway, 42, 0));
    reply.sequence = 43;
    assert(!cd_gateway_finish(&gateway, &reply, 1));
    reply.sequence = 42;
    reply.firmware_id ^= 1;
    assert(!cd_gateway_finish(&gateway, &reply, 1));
    reply.firmware_id = CD_FIRMWARE_ID;
    reply.flags = 0;
    assert(!cd_gateway_finish(&gateway, &reply, 1));
    cd_gateway_reset(&gateway);
    assert(!gateway.pending);
    assert(cd_gateway_begin(&gateway,42,0));
    reply.firmware_id=CD_MOTOR_FIRMWARE_ID;
    reply.flags=CD_MOTOR_IMPLEMENTED;
    assert(cd_gateway_finish(&gateway,&reply,1));
    puts("C gateway tests passed");
    return 0;
}
