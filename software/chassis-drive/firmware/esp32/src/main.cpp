#include <Arduino.h>
#include <WiFi.h>
#include "diagnostic_gateway.h"
#include "motion_bridge.h"

namespace {
constexpr char AP_NAME[] = "ChassisDrive-Setup";
constexpr char AP_PASSWORD[] = "Chassis2026";
constexpr uint16_t DIAGNOSTIC_PORT = 6001;
constexpr uint16_t MOTION_PORT = 6002;
constexpr uint32_t CLIENT_IDLE_MS = 1000;
constexpr uint32_t PARTIAL_FRAME_MS = 100;
constexpr int STM32_RX_PIN = 18; // STM32 PA2 TX -> ESP32 GPIO18 RX.
constexpr int STM32_TX_PIN = 17; // ESP32 GPIO17 TX -> STM32 PA3 RX.

enum class Mode { None, Diagnostic, Motion };

// Best-effort logs on both USB connectors; protocol data uses Serial2 only.
class DualConsole : public Print {
public:
    size_t write(uint8_t byte) override { return write(&byte, 1); }
    size_t write(const uint8_t *bytes, size_t length) override {
        Serial0.write(bytes, length);
        if (Serial) Serial.write(bytes, length);
        return length;
    }
};
DualConsole console;
WiFiServer diagnostic_server(DIAGNOSTIC_PORT, 1);
WiFiServer motion_server(MOTION_PORT, 1);
WiFiClient client;
Mode mode = Mode::None;
cm_bridge_t motion{};
cd_gateway_t diagnostic{};
cm_parser_t motion_tcp{}, motion_uart{};
cd_parser_t diagnostic_tcp{}, diagnostic_uart{};
uint32_t last_activity, frame_started, last_report;

void send_stop()
{
    // Universal STOP, never ARM/DRIVE. Its session=0 reply is never a PC ACK.
    const cm_request_t request{0, 0, 0, CM_STOP, 0, 0};
    uint8_t packet[CM_REQUEST_SIZE];
    cm_encode_request(&request, packet);
    if (Serial2.write(packet, sizeof packet) != sizeof packet)
        console.println("STOP send failed; STM32 command lease remains the fallback");
}

void reset_session()
{
    cm_bridge_reset(&motion);
    cd_gateway_reset(&diagnostic);
    cm_parser_reset(&motion_tcp);
    cm_parser_reset(&motion_uart);
    cd_parser_reset(&diagnostic_tcp);
    cd_parser_reset(&diagnostic_uart);
}

void close_session(const char *reason)
{
    if (mode == Mode::None) return;
    send_stop();
    client.stop();
    mode = Mode::None;
    reset_session();
    console.printf("Control disconnected: %s; STOP sent, reconnect manually\n", reason);
}

void accept_from(WiFiServer &server, Mode requested_mode)
{
    if (!server.hasClient()) return;
    WiFiClient candidate = server.available();
    if (!candidate) return;
    if (mode != Mode::None) { candidate.stop(); return; }
    reset_session();
    // Clear old bytes, then STOP. Its later UART reply is explicitly ignored.
    for (unsigned budget = 0; budget < 512 && Serial2.available(); ++budget) Serial2.read();
    send_stop();
    client = candidate;
    client.setNoDelay(true);
    mode = requested_mode;
    frame_started = last_activity = millis();
    console.println(mode == Mode::Motion
        ? "PC connected: HELLO -> ARM -> DRIVE; STM32 must confirm every request"
        : "Diagnostic client connected: PING/STOP only");
}

bool send_to_pc(const uint8_t *packet, size_t length)
{
    if (client.write(packet, length) != length) {
        close_session("TCP write failed");
        return false;
    }
    return true;
}

void receive_uart()
{
    for (unsigned budget = 0; budget < 128 && Serial2.available(); ++budget) {
        const uint8_t byte = static_cast<uint8_t>(Serial2.read());
        if (mode == Mode::Motion) {
            cm_reply_t reply;
            if (!cm_reply_feed(&motion_uart, byte, &reply)) continue;
            if (!reply.session && !reply.sequence) continue; // Untracked STOP.
            if (!cm_bridge_finish(&motion, &reply, millis())) {
                close_session("wrong, unsolicited or late STM32 motion reply");
                return;
            }
            uint8_t packet[CM_REPLY_SIZE];
            cm_encode_reply(&reply, packet);
            if (!send_to_pc(packet, sizeof packet)) return;
        } else if (mode == Mode::Diagnostic) {
            cd_reply_t reply;
            if (!cd_reply_feed(&diagnostic_uart, byte, &reply)) continue;
            if (!cd_gateway_finish(&diagnostic, &reply, millis())) {
                close_session("wrong or late STM32 diagnostic reply");
                return;
            }
            uint8_t packet[CD_REPLY_SIZE];
            cd_encode_reply(&reply, packet);
            if (!send_to_pc(packet, sizeof packet)) return;
        }
        // No client: discard bytes instead of saving them for a future client.
    }
}

void receive_tcp()
{
    for (unsigned budget = 0; budget < 64 && client.available(); ++budget) {
        if (motion.pending || diagnostic.pending) {
            close_session("pipelined commands; wait for STM32 confirmation first");
            return;
        }
        if (mode == Mode::Motion) {
            if (!motion_tcp.used) frame_started = millis();
            cm_request_t request;
            if (!cm_request_feed(&motion_tcp, static_cast<uint8_t>(client.read()), &request)) continue;
            if (!cm_bridge_begin(&motion, &request, millis())) {
                close_session("invalid duty, stale session/sequence or missing HELLO/ARM");
                return;
            }
            uint8_t packet[CM_REQUEST_SIZE];
            cm_encode_request(&request, packet);
            if (Serial2.write(packet, sizeof packet) != sizeof packet) {
                close_session("UART write failed"); return;
            }
        } else {
            if (!diagnostic_tcp.used) frame_started = millis();
            cd_request_t request;
            if (!cd_request_feed(&diagnostic_tcp, static_cast<uint8_t>(client.read()), &request)) continue;
            if (!cd_gateway_begin(&diagnostic, request.sequence, millis())) {
                close_session("diagnostic gateway busy"); return;
            }
            uint8_t packet[CD_REQUEST_SIZE];
            cd_encode_request(&request, packet);
            if (Serial2.write(packet, sizeof packet) != sizeof packet) {
                close_session("UART write failed"); return;
            }
        }
        last_activity = millis(); // Only a complete valid request renews PC activity.
    }
}

void check_timeouts()
{
    const uint32_t now = millis();
    if (cm_bridge_expired(&motion, now) || cd_gateway_expired(&diagnostic, now))
        close_session("STM32 confirmation timeout");
    else if ((motion_tcp.used || diagnostic_tcp.used) &&
             static_cast<uint32_t>(now-frame_started) >= PARTIAL_FRAME_MS)
        close_session("incomplete or corrupt PC frame");
    else if (static_cast<uint32_t>(now-last_activity) >= CLIENT_IDLE_MS)
        close_session("PC heartbeat stopped");
}

void report()
{
    console.printf("CHASSIS_BRIDGE v3 20261022 SSID=%s IP=%s motion=%u diag=%u active=%u pending=%u\n",
                   AP_NAME, WiFi.softAPIP().toString().c_str(), MOTION_PORT,
                   DIAGNOSTIC_PORT, static_cast<unsigned>(mode),
                   static_cast<unsigned>(motion.pending || diagnostic.pending));
}
} // namespace

void setup()
{
    Serial0.begin(115200);
    Serial.begin(115200); // Do not wait for USB monitor attachment.
    Serial2.setRxBufferSize(256);
    Serial2.begin(115200, SERIAL_8N1, STM32_RX_PIN, STM32_TX_PIN);
    reset_session();
    send_stop();
    console.println("\nCHASSIS_BRIDGE v3 20261022; boot keeps motion disabled");
    console.printf("Flash=%lu PSRAM=%lu UART2=115200 8N1 TX=17 RX=18\n",
                   static_cast<unsigned long>(ESP.getFlashChipSize()),
                   static_cast<unsigned long>(ESP.getPsramSize()));
    if (ESP.getFlashChipSize() != 16u * 1024u * 1024u) {
        console.println("Expected ESP32-S3 N16R8 with 16MB Flash; service not started");
        for (;;) delay(1000);
    }
    WiFi.mode(WIFI_AP);
    WiFi.setSleep(false);
    const IPAddress ip(192, 168, 4, 1);
    if (!WiFi.softAPConfig(ip, ip, IPAddress(255, 255, 255, 0)) ||
        !WiFi.softAP(AP_NAME, AP_PASSWORD, 6, false, 1)) {
        console.println("AP creation failed");
        for (;;) delay(1000);
    }
    diagnostic_server.begin(); diagnostic_server.setNoDelay(true);
    motion_server.begin(); motion_server.setNoDelay(true);
    console.printf("Wi-Fi password=%s; run PC control, then connect and enable\n", AP_PASSWORD);
    report(); last_report = millis();
}

void loop()
{
    if (mode != Mode::None && !client.connected()) close_session("TCP disconnected");
    accept_from(motion_server, Mode::Motion);
    accept_from(diagnostic_server, Mode::Diagnostic);
    receive_uart();
    if (mode != Mode::None) {
        check_timeouts();
        if (mode != Mode::None) receive_tcp();
    }
    if (static_cast<uint32_t>(millis()-last_report) >= 5000) {
        report(); last_report = millis();
    }
    delay(1);
}
