#ifndef CHASSIS_DIAGNOSTIC_PROTOCOL_H
#define CHASSIS_DIAGNOSTIC_PROTOCOL_H

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

#define CD_REQUEST_SIZE 16u
#define CD_REPLY_SIZE 24u
#define CD_FIRMWARE_ID UINT32_C(0x20261007)
#define CD_NO_MOTOR_IMPLEMENTATION UINT32_C(1)
#define CD_MOTOR_FIRMWARE_ID UINT32_C(0x20261011)
#define CD_MOTOR_IMPLEMENTED UINT32_C(2)

typedef enum { CD_PING = 1, CD_STOP = 2 } cd_opcode_t;
typedef struct { uint32_t sequence; cd_opcode_t opcode; } cd_request_t;
typedef struct {
    uint32_t sequence;
    uint32_t uptime_ms;
    uint32_t firmware_id;
    uint32_t flags;
} cd_reply_t;
typedef struct { uint8_t bytes[CD_REPLY_SIZE]; size_t used; } cd_parser_t;

uint32_t cd_crc32(const uint8_t *bytes, size_t length);
void cd_encode_request(const cd_request_t *request, uint8_t output[CD_REQUEST_SIZE]);
bool cd_decode_request(const uint8_t input[CD_REQUEST_SIZE], cd_request_t *request);
void cd_encode_reply(const cd_reply_t *reply, uint8_t output[CD_REPLY_SIZE]);
bool cd_decode_reply(const uint8_t input[CD_REPLY_SIZE], cd_reply_t *reply);
void cd_parser_reset(cd_parser_t *parser);
bool cd_request_feed(cd_parser_t *parser, uint8_t byte, cd_request_t *request);
bool cd_reply_feed(cd_parser_t *parser, uint8_t byte, cd_reply_t *reply);

#ifdef __cplusplus
}
#endif
#endif
