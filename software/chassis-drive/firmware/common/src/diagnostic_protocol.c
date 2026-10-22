#include "diagnostic_protocol.h"
#include <string.h>

static uint32_t read_u32(const uint8_t *p)
{
    return (uint32_t)p[0] | ((uint32_t)p[1] << 8) |
           ((uint32_t)p[2] << 16) | ((uint32_t)p[3] << 24);
}

static void write_u32(uint8_t *p, uint32_t value)
{
    for (unsigned i = 0; i < 4; ++i) p[i] = (uint8_t)(value >> (8u * i));
}

uint32_t cd_crc32(const uint8_t *bytes, size_t length)
{
    uint32_t crc = UINT32_C(0xFFFFFFFF);
    for (size_t i = 0; i < length; ++i) {
        crc ^= bytes[i];
        for (unsigned bit = 0; bit < 8; ++bit)
            crc = (crc >> 1) ^ ((crc & 1u) ? UINT32_C(0xEDB88320) : 0u);
    }
    return crc ^ UINT32_C(0xFFFFFFFF);
}

void cd_encode_request(const cd_request_t *request, uint8_t output[CD_REQUEST_SIZE])
{
    memcpy(output, "CDP1", 4);
    write_u32(output + 4, request->sequence);
    output[8] = (uint8_t)request->opcode;
    memset(output + 9, 0, 3);
    write_u32(output + 12, cd_crc32(output, 12));
}

bool cd_decode_request(const uint8_t input[CD_REQUEST_SIZE], cd_request_t *request)
{
    if (memcmp(input, "CDP1", 4) != 0 ||
        (input[8] != CD_PING && input[8] != CD_STOP) ||
        input[9] || input[10] || input[11] ||
        read_u32(input + 12) != cd_crc32(input, 12)) return false;
    request->sequence = read_u32(input + 4);
    request->opcode = (cd_opcode_t)input[8];
    return true;
}

void cd_encode_reply(const cd_reply_t *reply, uint8_t output[CD_REPLY_SIZE])
{
    memcpy(output, "CDS1", 4);
    write_u32(output + 4, reply->sequence);
    write_u32(output + 8, reply->uptime_ms);
    write_u32(output + 12, reply->firmware_id);
    write_u32(output + 16, reply->flags);
    write_u32(output + 20, cd_crc32(output, 20));
}

bool cd_decode_reply(const uint8_t input[CD_REPLY_SIZE], cd_reply_t *reply)
{
    if (memcmp(input, "CDS1", 4) != 0 ||
        read_u32(input + 20) != cd_crc32(input, 20)) return false;
    reply->sequence = read_u32(input + 4);
    reply->uptime_ms = read_u32(input + 8);
    reply->firmware_id = read_u32(input + 12);
    reply->flags = read_u32(input + 16);
    return true;
}

void cd_parser_reset(cd_parser_t *parser) { parser->used = 0; }

static void discard_first(cd_parser_t *parser)
{
    --parser->used;
    memmove(parser->bytes, parser->bytes + 1, parser->used);
}

bool cd_request_feed(cd_parser_t *parser, uint8_t byte, cd_request_t *request)
{
    if (parser->used >= CD_REQUEST_SIZE) cd_parser_reset(parser);
    parser->bytes[parser->used++] = byte;
    if (parser->used < CD_REQUEST_SIZE) return false;
    if (cd_decode_request(parser->bytes, request)) {
        cd_parser_reset(parser);
        return true;
    }
    discard_first(parser);
    return false;
}

bool cd_reply_feed(cd_parser_t *parser, uint8_t byte, cd_reply_t *reply)
{
    if (parser->used >= CD_REPLY_SIZE) cd_parser_reset(parser);
    parser->bytes[parser->used++] = byte;
    if (parser->used < CD_REPLY_SIZE) return false;
    if (cd_decode_reply(parser->bytes, reply)) {
        cd_parser_reset(parser);
        return true;
    }
    discard_first(parser);
    return false;
}
