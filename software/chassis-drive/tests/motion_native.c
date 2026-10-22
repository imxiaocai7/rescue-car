#include "motion_control.h"
#include <assert.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
static cm_request_t command(uint32_t seq,uint32_t session,uint32_t token,cm_opcode_t op,int l,int r) {
    return (cm_request_t){seq,session,token,op,(int16_t)l,(int16_t)r};
}
static uint32_t arm(cm_control_t *c,uint32_t now) {
    cm_request_t r=command(10,123,0,CM_HELLO,0,0);
    assert(cm_control_request(c,&r,now)==CM_OK);
    uint32_t token=c->token;
    r=command(11,123,token,CM_ARM,0,0);
    assert(cm_control_request(c,&r,now)==CM_OK);
    assert(c->flags==CM_ARMED); return token;
}
int main(int argc,char **argv) {
    if (argc==2) {
        if (strlen(argv[1])!=CM_REQUEST_SIZE*2) return 2;
        uint8_t packet[CM_REQUEST_SIZE];
        for (unsigned i=0;i<CM_REQUEST_SIZE;++i) {
            char pair[3]={argv[1][2*i],argv[1][2*i+1],0};
            packet[i]=(uint8_t)strtoul(pair,NULL,16);
        }
        cm_request_t r;
        if (!cm_decode_request(packet,&r)) return 2;
        cm_control_t c; cm_control_init(&c,1234,0);
        cm_result_t result=cm_control_request(&c,&r,50);
        cm_reply_t reply; cm_control_reply(&c,&r,50,result,&reply);
        uint8_t out[CM_REPLY_SIZE]; cm_encode_reply(&reply,out);
        for (unsigned i=0;i<CM_REPLY_SIZE;++i) printf("%02x",out[i]);
        puts(""); return 0;
    }
    cm_control_t c;
    cm_control_init(&c,1234,0);
    cm_request_t r=command(1,123,1235,CM_DRIVE,100,100);
    assert(cm_control_request(&c,&r,0)==CM_BAD_SESSION);
    assert(!c.flags && !c.output[0]);
    uint32_t token=arm(&c,0);
    r=command(12,123,token,CM_DRIVE,100,-100);
    assert(cm_control_request(&c,&r,0)==CM_OK);
    for (uint32_t t=10;t<=50;t+=10) cm_control_tick(&c,t);
    assert(c.output[0]==100 && c.output[1]==-100);
    assert(cm_control_request(&c,&r,200)==CM_OLD_SEQUENCE);
    r.sequence=13; r.token^=1;
    assert(cm_control_request(&c,&r,250)==CM_BAD_SESSION);
    cm_control_tick(&c,299); assert(c.flags==CM_ARMED);
    cm_control_tick(&c,300); assert(c.flags==CM_TIMED_OUT && !c.token && !c.output[0] && !c.output[1]);
    r.token=token; r.sequence=14;
    assert(cm_control_request(&c,&r,301)==CM_BAD_SESSION);
    token=arm(&c,400);
    r=command(12,123,token,CM_DRIVE,251,0);
    assert(cm_control_request(&c,&r,500)==CM_BAD_ARGUMENT);
    cm_control_tick(&c,700); assert(c.flags==CM_TIMED_OUT); /* Invalid traffic did not renew. */
    token=arm(&c,800);
    r=command(12,123,token,CM_STATUS,0,0);
    assert(cm_control_request(&c,&r,1000)==CM_OK);
    cm_control_tick(&c,1100); assert(c.flags==CM_TIMED_OUT); /* Status is not heartbeat. */
    token=arm(&c,1200);
    r=command(12,123,token,CM_DRIVE,100,100);
    assert(cm_control_request(&c,&r,1200)==CM_OK);
    for (uint32_t t=1210;t<=1250;t+=10) cm_control_tick(&c,t);
    r=command(13,123,token,CM_DRIVE,-100,100);
    assert(cm_control_request(&c,&r,1250)==CM_OK);
    for (uint32_t t=1260;t<=1300;t+=10) cm_control_tick(&c,t);
    assert(!c.output[0] && c.output[1]==100);
    cm_control_tick(&c,1310); assert(!c.output[0]);
    cm_control_tick(&c,1320); assert(c.output[0]==-20);
    r=command(1,999,0,CM_STOP,0,0);
    assert(cm_control_request(&c,&r,1321)==CM_OK && !c.flags && !c.token && !c.output[0]);
    token=arm(&c,1400);
    r=command(0,999,0,CM_HELLO,0,0);
    assert(cm_control_request(&c,&r,1401)==CM_OK && !c.flags && c.token!=token);
    r=command(12,123,token,CM_ARM,0,0);
    assert(cm_control_request(&c,&r,1402)==CM_BAD_SESSION); /* Old connection cannot rearm. */
    cm_control_disable(&c,CM_UART_FAULT);
    assert(!c.token && c.flags==CM_UART_FAULT);
    cm_control_init(&c,99999,0);
    assert(arm(&c,0)!=token); /* Fresh boot seed changes the challenge. */
    cm_control_init(&c,1234,UINT32_C(0xFFFFFF00));
    token=arm(&c,UINT32_C(0xFFFFFF00));
    cm_control_tick(&c,43); assert(c.flags==CM_ARMED);
    cm_control_tick(&c,44); assert(c.flags==CM_TIMED_OUT);
    cm_control_init(&c,1234,0);
    r=command(UINT32_C(0xFFFFFFFF),123,0,CM_HELLO,0,0);
    assert(cm_control_request(&c,&r,0)==CM_OK);
    r=command(0,123,c.token,CM_ARM,0,0);
    assert(cm_control_request(&c,&r,0)==CM_OK);
    uint8_t packet[CM_REQUEST_SIZE]; cm_encode_request(&r,packet);
    cm_request_t decoded; cm_parser_t p={{0},0};
    for (unsigned i=0;i<1000;++i) assert(!cm_request_feed(&p,0xFF,&decoded));
    unsigned found=0;
    for (unsigned i=0;i<CM_REQUEST_SIZE;++i) found+=cm_request_feed(&p,packet[i],&decoded);
    assert(found==1 && decoded.sequence==0 && decoded.token==r.token);
    for (unsigned i=0;i<CM_REQUEST_SIZE;++i) {
        packet[i]^=1; assert(!cm_decode_request(packet,&decoded)); packet[i]^=1;
    }
    cm_reply_t reply; cm_control_reply(&c,&r,0,CM_OK,&reply);
    uint8_t response[CM_REPLY_SIZE]; cm_encode_reply(&reply,response);
    cm_reply_t out;
    for (unsigned i=0;i<CM_REPLY_SIZE;++i) {
        response[i]^=1; assert(!cm_decode_reply(response,&out)); response[i]^=1;
    }
    assert(cm_decode_reply(response,&out) && out.flags==CM_ARMED);
    puts("C motion protocol/control tests passed"); return 0;
}
