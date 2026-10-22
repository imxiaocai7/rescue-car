#ifndef CHASSIS_MOTION_PROTOCOL_H
#define CHASSIS_MOTION_PROTOCOL_H
#include "diagnostic_protocol.h"
#ifdef __cplusplus
extern "C" {
#endif
#define CM_REQUEST_SIZE 32u
#define CM_REPLY_SIZE 40u
#define CM_FIRMWARE_ID UINT32_C(0x20261011)
#define CM_LEASE_MS 300u
#define CM_MAX_DUTY 250
#define CM_ARMED 1u
#define CM_TIMED_OUT 2u
#define CM_UART_FAULT 4u
typedef enum { CM_HELLO=1, CM_ARM=2, CM_DRIVE=3, CM_STOP=4, CM_STATUS=5 } cm_opcode_t;
typedef enum { CM_OK=0, CM_BAD_SESSION=1, CM_OLD_SEQUENCE=2,
               CM_NOT_ARMED=3, CM_BAD_ARGUMENT=4 } cm_result_t;
typedef struct {
    uint32_t sequence, session, token;
    cm_opcode_t opcode;
    int16_t left, right;
} cm_request_t;
typedef struct {
    uint32_t sequence, session, uptime_ms, firmware_id, flags, token;
    int16_t left, right;
    cm_result_t result;
} cm_reply_t;
typedef struct { uint8_t bytes[CM_REPLY_SIZE]; size_t used; } cm_parser_t;
void cm_encode_request(const cm_request_t *, uint8_t out[CM_REQUEST_SIZE]);
bool cm_decode_request(const uint8_t in[CM_REQUEST_SIZE], cm_request_t *);
void cm_encode_reply(const cm_reply_t *, uint8_t out[CM_REPLY_SIZE]);
bool cm_decode_reply(const uint8_t in[CM_REPLY_SIZE], cm_reply_t *);
void cm_parser_reset(cm_parser_t *);
bool cm_request_feed(cm_parser_t *, uint8_t, cm_request_t *);
bool cm_reply_feed(cm_parser_t *, uint8_t, cm_reply_t *);
#ifdef __cplusplus
}
#endif
#endif
