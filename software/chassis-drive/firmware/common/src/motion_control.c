#include "motion_control.h"
#include <string.h>
void cm_control_init(cm_control_t *c, uint32_t boot_random, uint32_t now) {
    memset(c,0,sizeof *c); c->challenge=boot_random; c->ramp_ms=now;
}
void cm_control_disable(cm_control_t *c, uint32_t reason) {
    c->flags=reason; c->token=0;
    for (unsigned i=0;i<2;++i) { c->target[i]=0; c->output[i]=0; c->reversing[i]=false; }
}
void cm_control_tick(cm_control_t *c, uint32_t now) {
    if (!(c->flags & CM_ARMED)) return;
    if ((uint32_t)(now-c->last_command_ms)>=CM_LEASE_MS) {
        cm_control_disable(c,CM_TIMED_OUT); return;
    }
    if ((uint32_t)(now-c->ramp_ms)<10) return;
    c->ramp_ms=now; /* No catch-up jump after a delayed scheduler. */
    for (unsigned i=0;i<2;++i) {
        if (c->reversing[i]) {
            if ((int32_t)(now-c->reverse_until[i])<0) continue;
            c->reversing[i]=false;
        }
        int16_t desired=c->target[i];
        bool opposite=(c->output[i]>0 && desired<0) || (c->output[i]<0 && desired>0);
        if (opposite) desired=0;
        int delta=desired-c->output[i];
        if (delta>20) delta=20;
        if (delta<-20) delta=-20;
        c->output[i]=(int16_t)(c->output[i]+delta);
        if (opposite && !c->output[i]) { c->reversing[i]=true; c->reverse_until[i]=now+20; }
    }
}
cm_result_t cm_control_request(cm_control_t *c, const cm_request_t *r, uint32_t now) {
    cm_control_tick(c,now);
    if (r->opcode<CM_HELLO || r->opcode>CM_STATUS) return CM_BAD_ARGUMENT;
    if (r->opcode==CM_STOP) {
        /* Any valid STOP can remove energy, even after a lost session. */
        cm_control_disable(c,0); return CM_OK;
    }
    if (r->opcode!=CM_DRIVE && (r->left || r->right)) return CM_BAD_ARGUMENT;
    if (r->opcode==CM_HELLO) {
        if (!r->session || r->token) return CM_BAD_ARGUMENT;
        cm_control_disable(c,0);
        if (++c->challenge==0) ++c->challenge;
        c->session=r->session; c->token=c->challenge; c->sequence=r->sequence;
        return CM_OK;
    }
    if (!c->token || r->session!=c->session || r->token!=c->token) return CM_BAD_SESSION;
    uint32_t distance=r->sequence-c->sequence;
    if (!distance || distance>=UINT32_C(0x80000000)) return CM_OLD_SEQUENCE;
    if (r->opcode==CM_ARM) {
        if (c->flags & CM_ARMED) return CM_BAD_ARGUMENT;
        c->flags=CM_ARMED; c->last_command_ms=now; c->ramp_ms=now;
    } else if (r->opcode==CM_DRIVE) {
        if (!(c->flags & CM_ARMED)) return CM_NOT_ARMED;
        if (r->left>CM_MAX_DUTY || r->left<-CM_MAX_DUTY ||
            r->right>CM_MAX_DUTY || r->right<-CM_MAX_DUTY) return CM_BAD_ARGUMENT;
        c->target[0]=r->left; c->target[1]=r->right; c->last_command_ms=now;
    }
    /* STATUS consumes a sequence but never extends the movement lease. */
    c->sequence=r->sequence; return CM_OK;
}
void cm_control_reply(const cm_control_t *c, const cm_request_t *r, uint32_t now,
                      cm_result_t result, cm_reply_t *reply) {
    *reply=(cm_reply_t){r->sequence,r->session,now,CM_FIRMWARE_ID,c->flags,c->token,
                       c->output[0],c->output[1],result};
}
