#include "diagnostic_protocol.h"
#include <assert.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

int main(int argc, char **argv)
{
    assert(cd_crc32((const uint8_t *)"123456789", 9) == UINT32_C(0xCBF43926));
    if (argc == 2) {
        assert(strlen(argv[1]) == CD_REQUEST_SIZE * 2u);
        uint8_t packet[CD_REQUEST_SIZE];
        for (unsigned i = 0; i < CD_REQUEST_SIZE; ++i) {
            char pair[3] = {argv[1][i * 2], argv[1][i * 2 + 1], 0};
            packet[i] = (uint8_t)strtoul(pair, NULL, 16);
        }
        cd_request_t request;
        if (!cd_decode_request(packet, &request)) return 2;
        cd_reply_t reply = {request.sequence, 1234, CD_FIRMWARE_ID, CD_NO_MOTOR_IMPLEMENTATION};
        uint8_t result[CD_REPLY_SIZE];
        cd_encode_reply(&reply, result);
        for (unsigned i = 0; i < CD_REPLY_SIZE; ++i) printf("%02x", result[i]);
        puts("");
        return 0;
    }
    cd_parser_t parser = {{0}, 0};
    cd_request_t request = {UINT32_C(0xFFFFFFFF), CD_STOP}, decoded;
    uint8_t packet[CD_REQUEST_SIZE];
    cd_encode_request(&request, packet);
    for (unsigned i = 0; i < 200; ++i) assert(!cd_request_feed(&parser, 0xFF, &decoded));
    unsigned found = 0;
    for (unsigned i = 0; i < CD_REQUEST_SIZE; ++i) found += cd_request_feed(&parser, packet[i], &decoded);
    assert(found == 1 && decoded.sequence == request.sequence && decoded.opcode == CD_STOP);
    for (unsigned i = 0; i < CD_REQUEST_SIZE; ++i) {
        packet[i] ^= 1;
        assert(!cd_decode_request(packet, &decoded));
        packet[i] ^= 1;
    }
    cd_reply_t reply = {42, 10, CD_FIRMWARE_ID, CD_NO_MOTOR_IMPLEMENTATION}, decoded_reply;
    uint8_t response[CD_REPLY_SIZE];
    cd_encode_reply(&reply, response);
    cd_parser_reset(&parser);
    found = 0;
    for (unsigned i = 0; i < CD_REPLY_SIZE; ++i) found += cd_reply_feed(&parser, response[i], &decoded_reply);
    assert(found == 1 && decoded_reply.sequence == 42 && decoded_reply.uptime_ms == 10);
    puts("C diagnostic protocol tests passed");
    return 0;
}
