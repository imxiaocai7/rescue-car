#include <Arduino.h>
#include <WiFi.h>

#include <esp_heap_caps.h>

namespace {

constexpr uint32_t kCameraBaud = 921600;
constexpr int kCameraRxPin = 16;  // OpenMV P4 TX -> ESP32-S3 GPIO16 RX
constexpr int kCameraTxPin = 15;  // OpenMV P5 RX <- ESP32-S3 GPIO15 TX
constexpr uint16_t kTcpPort = 5000;
constexpr uint16_t kMotionTcpPort = 5001;
constexpr uint32_t kMotionBaud = 115200;
constexpr int kMotionRxPin = 18;  // STM32 TX -> ESP32-S3 GPIO18 RX (verify board pinout)
constexpr int kMotionTxPin = 17;  // STM32 RX <- ESP32-S3 GPIO17 TX (verify board pinout)
constexpr size_t kMotionBridgeBufferSize = 256;
constexpr size_t kHeaderSize = 26;
constexpr size_t kMaxJpegSize = 120000;
constexpr uint16_t kMaxImageSide = 2048;
constexpr uint32_t kUartInterByteTimeoutMs = 1500;
constexpr uint32_t kTcpWriteTimeoutMs = 3000;
constexpr uint32_t kTcpAckTimeoutMs = 15000;

constexpr char kApSsid[] = "RescueBot-S3";
constexpr char kApPassword[] = "rescue123";
constexpr uint8_t kApChannel = 6;

constexpr uint8_t kMagic[4] = {'R', 'S', 'V', '1'};
constexpr uint8_t kAckMagic[4] = {'A', 'C', 'K', '1'};
constexpr uint8_t kVersion = 1;
constexpr uint8_t kJpegKind = 1;

WiFiServer server(kTcpPort);
WiFiServer motionServer(kMotionTcpPort);
HardwareSerial cameraUart(1);
HardwareSerial motionUart(2);
uint8_t *jpegBuffer = nullptr;

struct FrameHeader {
  uint8_t bytes[kHeaderSize];
  uint32_t sequence;
  uint32_t cameraTicksMs;
  uint16_t width;
  uint16_t height;
  uint32_t jpegSize;
  uint32_t jpegCrc32;
};

uint16_t readLe16(const uint8_t *p) {
  return static_cast<uint16_t>(p[0]) |
         (static_cast<uint16_t>(p[1]) << 8);
}

uint32_t readLe32(const uint8_t *p) {
  return static_cast<uint32_t>(p[0]) |
         (static_cast<uint32_t>(p[1]) << 8) |
         (static_cast<uint32_t>(p[2]) << 16) |
         (static_cast<uint32_t>(p[3]) << 24);
}

bool parseHeader(const uint8_t *bytes, FrameHeader &header) {
  if (memcmp(bytes, kMagic, sizeof(kMagic)) != 0 ||
      bytes[4] != kVersion || bytes[5] != kJpegKind) {
    return false;
  }

  header.sequence = readLe32(bytes + 6);
  header.cameraTicksMs = readLe32(bytes + 10);
  header.width = readLe16(bytes + 14);
  header.height = readLe16(bytes + 16);
  header.jpegSize = readLe32(bytes + 18);
  header.jpegCrc32 = readLe32(bytes + 22);
  memcpy(header.bytes, bytes, kHeaderSize);

  return header.width > 0 && header.width <= kMaxImageSide &&
         header.height > 0 && header.height <= kMaxImageSide &&
         header.jpegSize >= 2 && header.jpegSize <= kMaxJpegSize;
}

bool readUartExact(uint8_t *destination, size_t length) {
  size_t received = 0;
  uint32_t lastByteAt = millis();

  while (received < length) {
    int available = cameraUart.available();
    if (available > 0) {
      size_t request = min(static_cast<size_t>(available), length - received);
      int count = cameraUart.read(destination + received, request);
      if (count > 0) {
        received += static_cast<size_t>(count);
        lastByteAt = millis();
      }
    } else {
      if (millis() - lastByteAt >= kUartInterByteTimeoutMs) {
        return false;
      }
      delay(1);
    }
  }
  return true;
}

bool readNextHeader(FrameHeader &header) {
  uint8_t candidate[kHeaderSize];
  if (!readUartExact(candidate, sizeof(candidate))) {
    return false;
  }
  if (parseHeader(candidate, header)) {
    return true;
  }

  // Recover from startup noise or a partial UART frame by sliding over bytes.
  uint32_t lastByteAt = millis();
  while (millis() - lastByteAt < kUartInterByteTimeoutMs) {
    if (cameraUart.available() <= 0) {
      delay(1);
      continue;
    }
    int next = cameraUart.read();
    if (next < 0) {
      continue;
    }
    memmove(candidate, candidate + 1, sizeof(candidate) - 1);
    candidate[sizeof(candidate) - 1] = static_cast<uint8_t>(next);
    lastByteAt = millis();
    if (parseHeader(candidate, header)) {
      return true;
    }
  }
  return false;
}

uint32_t crc32(const uint8_t *data, size_t length) {
  uint32_t crc = 0xFFFFFFFFUL;
  for (size_t i = 0; i < length; ++i) {
    crc ^= data[i];
    for (uint8_t bit = 0; bit < 8; ++bit) {
      crc = (crc >> 1) ^ ((crc & 1U) ? 0xEDB88320UL : 0U);
    }
  }
  return crc ^ 0xFFFFFFFFUL;
}

bool receiveFrame(FrameHeader &header) {
  if (!readNextHeader(header)) {
    Serial.println("Camera UART: header timeout or invalid frame");
    return false;
  }
  if (!readUartExact(jpegBuffer, header.jpegSize)) {
    Serial.println("Camera UART: JPEG body timeout");
    return false;
  }

  if (jpegBuffer[0] != 0xFF || jpegBuffer[1] != 0xD8 ||
      jpegBuffer[header.jpegSize - 2] != 0xFF ||
      jpegBuffer[header.jpegSize - 1] != 0xD9) {
    Serial.println("Camera UART: JPEG markers invalid");
    return false;
  }
  if (crc32(jpegBuffer, header.jpegSize) != header.jpegCrc32) {
    Serial.println("Camera UART: CRC32 mismatch");
    return false;
  }
  return true;
}

bool writeTcpExact(WiFiClient &client, const uint8_t *data, size_t length) {
  size_t sent = 0;
  uint32_t lastProgressAt = millis();
  while (sent < length) {
    if (!client.connected()) {
      return false;
    }
    size_t chunk = min(static_cast<size_t>(1024), length - sent);
    size_t count = client.write(data + sent, chunk);
    if (count > 0) {
      sent += count;
      lastProgressAt = millis();
    } else {
      if (millis() - lastProgressAt >= kTcpWriteTimeoutMs) {
        return false;
      }
      delay(1);
    }
  }
  return true;
}

bool forwardFrame(WiFiClient &client, const FrameHeader &header) {
  if (!writeTcpExact(client, header.bytes, sizeof(header.bytes))) {
    return false;
  }
  return writeTcpExact(client, jpegBuffer, header.jpegSize);
}

bool waitForFrameAck(WiFiClient &client, uint32_t expectedSequence) {
  uint8_t ack[8];
  size_t received = 0;
  const uint32_t startedAt = millis();
  while (received < sizeof(ack)) {
    if (!client.connected() || millis() - startedAt >= kTcpAckTimeoutMs) {
      return false;
    }
    int available = client.available();
    if (available > 0) {
      size_t request = min(static_cast<size_t>(available), sizeof(ack) - received);
      int count = client.read(ack + received, request);
      if (count > 0) {
        received += static_cast<size_t>(count);
      }
    } else {
      delay(1);
    }
  }
  return memcmp(ack, kAckMagic, sizeof(kAckMagic)) == 0 &&
         readLe32(ack + 4) == expectedSequence;
}

void printNetworkInfo() {
  Serial.print("Wi-Fi AP: ");
  Serial.println(kApSsid);
  Serial.print("AP IP: ");
  Serial.println(WiFi.softAPIP());
  Serial.printf("TCP image server: %s:%u\n", WiFi.softAPIP().toString().c_str(),
                kTcpPort);
  Serial.printf("TCP motion bridge: %s:%u\n", WiFi.softAPIP().toString().c_str(),
                kMotionTcpPort);
}

// USB diagnostics are accepted only while no image TCP session is active.
// They exercise the camera UART only; they never address the motion UART.
void cameraWiringTest() {
  cameraUart.end();
  cameraUart.begin(115200, SERIAL_8N1, kCameraRxPin, kCameraTxPin);
  while (cameraUart.available()) {
    cameraUart.read();
  }
  const char request[] = "RESCUE_PING\n";
  const char expected[] = "RESCUE_PONG\n";
  uint8_t reply[sizeof(expected) - 1];
  cameraUart.write(reinterpret_cast<const uint8_t *>(request), sizeof(request) - 1);
  const bool ok = readUartExact(reply, sizeof(reply)) &&
                  memcmp(reply, expected, sizeof(reply)) == 0;
  Serial.println(ok ? "UART_TEST_OK" : "UART_TEST_FAILED");
  cameraUart.end();
  cameraUart.begin(kCameraBaud, SERIAL_8N1, kCameraRxPin, kCameraTxPin);
}

bool writeUsbExact(const uint8_t *data, size_t length) {
  size_t sent = 0;
  uint32_t lastProgressAt = millis();
  while (sent < length) {
    if (!Serial || millis() - lastProgressAt >= kTcpWriteTimeoutMs) {
      return false;
    }
    const int available = Serial.availableForWrite();
    if (available <= 0) {
      delay(1);
      continue;
    }
    const size_t count = Serial.write(data + sent,
        min(length - sent, static_cast<size_t>(available)));
    if (count) {
      sent += count;
      lastProgressAt = millis();
    }
  }
  return true;
}

void handleUsbDiagnostic() {
  if (!Serial.available()) {
    return;
  }
  const int command = Serial.read();
  if (command == '?') {
    printNetworkInfo();
    Serial.printf("Flash=%u PSRAM=%u CameraRX=%d CameraTX=%d Baud=%lu\n",
        ESP.getFlashChipSize(), ESP.getPsramSize(), kCameraRxPin, kCameraTxPin,
        static_cast<unsigned long>(kCameraBaud));
    Serial.println("USB diagnostic: T=115200 wiring ping, J=one validated RSV1 frame");
  } else if (command == 'T') {
    cameraWiringTest();
  } else if (command == 'J') {
    const uint8_t request = 'R';
    FrameHeader header{};
    if (cameraUart.write(&request, 1) == 1 && receiveFrame(header)) {
      if (writeUsbExact(header.bytes, sizeof(header.bytes))) {
        writeUsbExact(jpegBuffer, header.jpegSize);
      }
    } else {
      Serial.println("USB_FRAME_FAILED");
    }
  }
}

void motionBridgeTask(void *parameter) {
  (void)parameter;
  uint8_t buffer[kMotionBridgeBufferSize];

  for (;;) {
    WiFiClient client = motionServer.available();
    if (!client) {
      vTaskDelay(pdMS_TO_TICKS(10));
      continue;
    }

    client.setNoDelay(true);
    Serial.printf("Motion client connected: %s\n",
                  client.remoteIP().toString().c_str());
    while (client.connected()) {
      bool progressed = false;

      const int tcpAvailable = client.available();
      if (tcpAvailable > 0) {
        const size_t request = min(
            static_cast<size_t>(tcpAvailable), sizeof(buffer));
        const int count = client.read(buffer, request);
        if (count > 0) {
          const size_t written = motionUart.write(
              buffer, static_cast<size_t>(count));
          if (written != static_cast<size_t>(count)) {
            Serial.println("Motion UART: incomplete TCP-to-UART write");
            break;
          }
          progressed = true;
        }
      }

      const int uartAvailable = motionUart.available();
      if (uartAvailable > 0) {
        const size_t request = min(
            static_cast<size_t>(uartAvailable), sizeof(buffer));
        const int count = motionUart.read(buffer, request);
        if (count > 0) {
          if (!writeTcpExact(client, buffer, static_cast<size_t>(count))) {
            Serial.println("Motion TCP client stopped reading telemetry");
            break;
          }
          progressed = true;
        }
      }

      if (!progressed) {
        vTaskDelay(pdMS_TO_TICKS(1));
      }
    }
    client.stop();
    Serial.println("Motion client disconnected");
  }
}

}  // namespace

void setup() {
  Serial.begin(115200);
  delay(300);
  Serial.println("RescueBot ESP32-S3 camera gateway starting");

  cameraUart.setRxBufferSize(8192);
  cameraUart.begin(kCameraBaud, SERIAL_8N1, kCameraRxPin, kCameraTxPin);
  motionUart.setRxBufferSize(2048);
  motionUart.begin(kMotionBaud, SERIAL_8N1, kMotionRxPin, kMotionTxPin);

  if (!psramFound()) {
    Serial.println("ERROR: PSRAM not detected; refusing to relay incomplete frames");
    while (true) {
      delay(1000);
    }
  }
  jpegBuffer = static_cast<uint8_t *>(ps_malloc(kMaxJpegSize));
  if (jpegBuffer == nullptr) {
    Serial.println("ERROR: cannot allocate JPEG frame buffer in PSRAM");
    while (true) {
      delay(1000);
    }
  }

  WiFi.mode(WIFI_AP);
  WiFi.setSleep(false);
  if (!WiFi.softAP(kApSsid, kApPassword, kApChannel, false, 1)) {
    Serial.println("ERROR: failed to start Wi-Fi access point");
    while (true) {
      delay(1000);
    }
  }

  server.begin();
  server.setNoDelay(true);
  motionServer.begin();
  printNetworkInfo();
  Serial.printf("Camera UART: %lu baud, RX GPIO%d, TX GPIO%d\n",
                static_cast<unsigned long>(kCameraBaud), kCameraRxPin,
                kCameraTxPin);
  Serial.printf("PSRAM frame buffer: %u bytes\n",
                static_cast<unsigned>(kMaxJpegSize));
  const BaseType_t taskCreated = xTaskCreatePinnedToCore(
      motionBridgeTask,
      "motion-bridge",
      4096,
      nullptr,
      1,
      nullptr,
      0);
  if (taskCreated != pdPASS) {
    Serial.println("ERROR: could not create motion bridge task");
  }
}

void loop() {
  WiFiClient client = server.available();
  if (!client) {
    handleUsbDiagnostic();
    delay(10);
    return;
  }

  client.setNoDelay(true);
  Serial.printf("PC connected: %s\n", client.remoteIP().toString().c_str());
  while (client.connected()) {
    const uint8_t request = 'R';
    if (cameraUart.write(&request, 1) != 1) {
      Serial.println("Camera UART: request write failed");
      break;
    }

    FrameHeader header{};
    if (!receiveFrame(header)) {
      // A damaged frame is dropped. The next request starts a fresh frame.
      continue;
    }
    if (!forwardFrame(client, header)) {
      Serial.println("TCP client disconnected or stopped reading");
      break;
    }
    if (!waitForFrameAck(client, header.sequence)) {
      Serial.printf("TCP ACK missing or invalid for frame #%lu\n",
                    static_cast<unsigned long>(header.sequence));
      break;
    }
    Serial.printf("Forwarded frame #%lu (%ux%u, %lu bytes)\n",
                  static_cast<unsigned long>(header.sequence), header.width,
                  header.height, static_cast<unsigned long>(header.jpegSize));
    delay(1);
  }

  client.stop();
  Serial.println("PC disconnected");
}
