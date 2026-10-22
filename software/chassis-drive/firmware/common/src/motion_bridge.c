#include "motion_bridge.h"
#include <string.h>

void cm_bridge_reset(cm_bridge_t *b) { memset(b, 0, sizeof *b); }

bool cm_bridge_expired(const cm_bridge_t *b, uint32_t now) {
    return b->pending && (uint32_t)(now-b->sent_ms) >= CM_BRIDGE_REPLY_TIMEOUT_MS;
}

bool cm_bridge_begin(cm_bridge_t *b, const cm_request_t *r, uint32_t now) {
    if (b->pending || !r->session || r->opcode<CM_HELLO || r->opcode>CM_STATUS)
        return false;
    if (r->left>CM_MAX_DUTY || r->left<-CM_MAX_DUTY ||
        r->right>CM_MAX_DUTY || r->right<-CM_MAX_DUTY ||
        (r->opcode!=CM_DRIVE && (r->left || r->right))) return false;
    if (r->opcode==CM_HELLO) {
        if (r->token) return false;
    } else if (r->opcode!=CM_STOP) {
        uint32_t distance=r->sequence-b->sequence;
        if (!b->established || !b->token || r->session!=b->session ||
            r->token!=b->token || !distance || distance>=UINT32_C(0x80000000))
            return false;
        if ((r->opcode==CM_ARM && b->armed) || (r->opcode==CM_DRIVE && !b->armed))
            return false;
    }
    b->request=*r; b->sent_ms=now; b->pending=true;
    return true;
}

bool cm_bridge_finish(cm_bridge_t *b, const cm_reply_t *r, uint32_t now) {
    if (!b->pending || cm_bridge_expired(b,now) ||
        r->sequence!=b->request.sequence || r->session!=b->request.session ||
        r->firmware_id!=CM_FIRMWARE_ID || r->flags & ~(CM_ARMED|CM_TIMED_OUT|CM_UART_FAULT) ||
        r->left>CM_MAX_DUTY || r->left<-CM_MAX_DUTY ||
        r->right>CM_MAX_DUTY || r->right<-CM_MAX_DUTY || r->result>CM_BAD_ARGUMENT)
        return false;
    if (b->have_ticks && (uint32_t)(r->uptime_ms-b->last_ticks)>=UINT32_C(0x80000000))
        return false; /* STM32 reset: explicit reconnect required. */
    if (r->result==CM_OK) {
        if (b->request.opcode==CM_HELLO) {
            if (!r->token || r->flags || r->left || r->right) return false;
        } else if (b->request.opcode==CM_STOP) {
            if (r->token || r->flags || r->left || r->right) return false;
        } else {
            if (r->token!=b->request.token) return false;
            if ((b->request.opcode==CM_ARM || b->request.opcode==CM_DRIVE) &&
                !(r->flags & CM_ARMED)) return false;
        }
    }
    b->pending=false; b->have_ticks=true; b->last_ticks=r->uptime_ms;
    b->sequence=b->request.sequence; b->session=b->request.session;
    b->token=r->token; b->armed=(r->flags & CM_ARMED)!=0;
    b->established=r->result==CM_OK && r->token!=0;
    return true; /* Forward real rejections too; PC must display them. */
}
