#include "motion_protocol.h"
#include <string.h>
static uint32_t r32(const uint8_t *p) {
    return (uint32_t)p[0] | (uint32_t)p[1]<<8 | (uint32_t)p[2]<<16 | (uint32_t)p[3]<<24;
}
static void w32(uint8_t *p, uint32_t n) {
    for (unsigned i=0; i<4; ++i) p[i]=(uint8_t)(n>>(8*i));
}
static int16_t r16(const uint8_t *p) {
    uint16_t n=(uint16_t)((uint16_t)p[0] | (uint16_t)p[1]<<8);
    return (int16_t)(n <= 32767 ? (int32_t)n : (int32_t)n-65536);
}
static void w16(uint8_t *p, int16_t n) { p[0]=(uint8_t)n; p[1]=(uint8_t)((uint16_t)n>>8); }
void cm_encode_request(const cm_request_t *r, uint8_t out[CM_REQUEST_SIZE]) {
    memset(out,0,CM_REQUEST_SIZE); memcpy(out,"DCM2",4);
    w32(out+4,r->sequence); w32(out+8,r->session); out[12]=(uint8_t)r->opcode;
    w32(out+16,r->token); w16(out+20,r->left); w16(out+22,r->right);
    w32(out+28,cd_crc32(out,28));
}
bool cm_decode_request(const uint8_t in[CM_REQUEST_SIZE], cm_request_t *r) {
    if (memcmp(in,"DCM2",4) || in[12]<CM_HELLO || in[12]>CM_STATUS ||
        in[13] || in[14] || in[15] || r32(in+24) || r32(in+28)!=cd_crc32(in,28)) return false;
    r->sequence=r32(in+4); r->session=r32(in+8); r->opcode=(cm_opcode_t)in[12];
    r->token=r32(in+16); r->left=r16(in+20); r->right=r16(in+22); return true;
}
void cm_encode_reply(const cm_reply_t *r, uint8_t out[CM_REPLY_SIZE]) {
    memset(out,0,CM_REPLY_SIZE); memcpy(out,"DCS2",4);
    w32(out+4,r->sequence); w32(out+8,r->session); w32(out+12,r->uptime_ms);
    w32(out+16,r->firmware_id); w32(out+20,r->flags); w32(out+24,r->token);
    w16(out+28,r->left); w16(out+30,r->right); out[32]=(uint8_t)r->result;
    w32(out+36,cd_crc32(out,36));
}
bool cm_decode_reply(const uint8_t in[CM_REPLY_SIZE], cm_reply_t *r) {
    if (memcmp(in,"DCS2",4) || in[32]>CM_BAD_ARGUMENT || in[33] || in[34] || in[35] ||
        r32(in+36)!=cd_crc32(in,36)) return false;
    r->sequence=r32(in+4); r->session=r32(in+8); r->uptime_ms=r32(in+12);
    r->firmware_id=r32(in+16); r->flags=r32(in+20); r->token=r32(in+24);
    r->left=r16(in+28); r->right=r16(in+30); r->result=(cm_result_t)in[32]; return true;
}
void cm_parser_reset(cm_parser_t *p) { p->used=0; }
static void shift(cm_parser_t *p) { --p->used; memmove(p->bytes,p->bytes+1,p->used); }
bool cm_request_feed(cm_parser_t *p, uint8_t b, cm_request_t *r) {
    if (p->used>=CM_REQUEST_SIZE) cm_parser_reset(p);
    p->bytes[p->used++]=b;
    if (p->used<CM_REQUEST_SIZE) return false;
    if (cm_decode_request(p->bytes,r)) { cm_parser_reset(p); return true; }
    shift(p); return false;
}
bool cm_reply_feed(cm_parser_t *p, uint8_t b, cm_reply_t *r) {
    if (p->used>=CM_REPLY_SIZE) cm_parser_reset(p);
    p->bytes[p->used++]=b;
    if (p->used<CM_REPLY_SIZE) return false;
    if (cm_decode_reply(p->bytes,r)) { cm_parser_reset(p); return true; }
    shift(p); return false;
}
