#include "motion_bridge.h"
#include "motion_control.h"
#include <assert.h>
#include <stdio.h>
#include <string.h>

static cm_reply_t respond(cm_control_t *c, const cm_request_t *q, uint32_t now) {
    cm_reply_t reply;
    cm_result_t result=cm_control_request(c,q,now);
    cm_control_reply(c,q,now,result,&reply);
    return reply;
}

static cm_reply_t exchange(cm_bridge_t *b, cm_control_t *c,
                            const cm_request_t *q, uint32_t now) {
    assert(cm_bridge_begin(b,q,now));
    cm_reply_t reply=respond(c,q,now);
    assert(cm_bridge_finish(b,&reply,now+1));
    return reply;
}

/* Stream mode lets the actual Python socket client exercise both C components.
   It emulates transport, not GPIO, RF timing or motor motion. */
static int stream(void) {
    cm_bridge_t b;
    cm_control_t c;
    cm_bridge_reset(&b); cm_control_init(&c,1234,0);
    char line[100];
    uint32_t now=0;
    while (fgets(line,sizeof line,stdin)) {
        cm_parser_t parser={{0},0};
        cm_request_t q;
        bool valid=false;
        for (unsigned i=0;i<CM_REQUEST_SIZE;++i) {
            unsigned value;
            if (sscanf(line+i*2,"%2x",&value)!=1) return 2;
            valid=cm_request_feed(&parser,(uint8_t)value,&q);
        }
        if (!valid || !cm_bridge_begin(&b,&q,now)) {
            puts("DROP"); fflush(stdout); return 1;
        }
        cm_reply_t reply=respond(&c,&q,now);
        if (!cm_bridge_finish(&b,&reply,now+1)) return 3;
        uint8_t packet[CM_REPLY_SIZE]; cm_encode_reply(&reply,packet);
        for (unsigned i=0;i<sizeof packet;++i) printf("%02x",packet[i]);
        puts(""); fflush(stdout); now+=50;
    }
    return 0;
}

int main(int argc, char **argv) {
    if (argc==2 && !strcmp(argv[1],"--stream")) return stream();
    cm_bridge_t b;
    cm_control_t c;
    cm_bridge_reset(&b); cm_control_init(&c,1234,0);
    cm_request_t q={1,42,0,CM_DRIVE,100,100};
    assert(!cm_bridge_begin(&b,&q,0)); /* No movement before handshake. */
    q.opcode=CM_HELLO; q.left=q.right=0; q.token=99;
    assert(!cm_bridge_begin(&b,&q,0));
    q.token=0; q.sequence=UINT32_MAX-1;
    assert(cm_bridge_begin(&b,&q,0));
    assert(!cm_bridge_begin(&b,&q,1)); /* Only one outstanding command. */
    cm_reply_t reply=respond(&c,&q,0), bad=reply;
    bad.sequence^=1; assert(!cm_bridge_finish(&b,&bad,1));
    bad=reply; bad.session^=1; assert(!cm_bridge_finish(&b,&bad,1));
    bad=reply; bad.firmware_id^=1; assert(!cm_bridge_finish(&b,&bad,1));
    bad=reply; bad.left=251; assert(!cm_bridge_finish(&b,&bad,1));
    bad=reply; bad.flags=8; assert(!cm_bridge_finish(&b,&bad,1));
    bad=reply; bad.token=0; assert(!cm_bridge_finish(&b,&bad,1));
    assert(cm_bridge_finish(&b,&reply,1));
    assert(b.established && !b.armed && b.token);
    assert(!cm_bridge_finish(&b,&reply,1)); /* Unsolicited/duplicate reply. */
    q.token=reply.token; q.sequence=UINT32_MAX; q.opcode=CM_ARM;
    reply=exchange(&b,&c,&q,10); assert(b.armed && reply.flags==CM_ARMED);
    q.sequence=0; q.opcode=CM_DRIVE; q.left=100; q.right=-100;
    reply=exchange(&b,&c,&q,20); assert(reply.result==CM_OK);
    assert(!cm_bridge_begin(&b,&q,21)); /* Replay, including wrapped sequence. */
    ++q.sequence; q.session=43; assert(!cm_bridge_begin(&b,&q,21));
    q.session=42; ++q.token; assert(!cm_bridge_begin(&b,&q,21)); --q.token;
    q.left=251; assert(!cm_bridge_begin(&b,&q,21)); q.left=100;
    q.opcode=CM_ARM; q.left=q.right=0; assert(!cm_bridge_begin(&b,&q,21));
    q.opcode=CM_STATUS;
    reply=exchange(&b,&c,&q,320); /* STATUS never renews motor lease. */
    assert(reply.result==CM_BAD_SESSION && !b.established && !b.armed);
    q.opcode=CM_ARM; ++q.sequence; assert(!cm_bridge_begin(&b,&q,321));
    q.opcode=CM_STOP; q.session=99; q.token=999;
    reply=exchange(&b,&c,&q,330); assert(!reply.token && !reply.flags);

    cm_bridge_reset(&b);
    q=(cm_request_t){1,42,0,CM_HELLO,0,0};
    assert(cm_bridge_begin(&b,&q,UINT32_C(0xFFFFFFF0)));
    reply=respond(&c,&q,340);
    assert(!cm_bridge_expired(&b,133));
    assert(cm_bridge_expired(&b,134)); /* 150ms boundary across tick wrap. */
    assert(!cm_bridge_finish(&b,&reply,134));
    cm_bridge_reset(&b);
    reply=exchange(&b,&c,&q,350);
    ++q.sequence; q.token=reply.token; q.opcode=CM_STATUS;
    assert(cm_bridge_begin(&b,&q,351));
    reply=respond(&c,&q,351); reply.uptime_ms=2;
    assert(!cm_bridge_finish(&b,&reply,352)); /* STM32 reboot, not normal wrap. */
    b.last_ticks=UINT32_C(0xFFFFFFF0); reply.uptime_ms=16;
    assert(cm_bridge_finish(&b,&reply,352));
    puts("C motion bridge tests passed");
    return 0;
}
