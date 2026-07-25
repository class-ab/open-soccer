// Teensy 4.1 - Dual/Multi-Color Tracking UART Receiver
// ------------------------------------------------------
// Receives 8-byte packets from an OpenMV Cam RT1062 over hardware
// UART (the Teensy's Serial7 peripheral) and decodes them into
// per-color ball-tracking data (detected flag, angle, radius, size).
//
// WIRING (3 wires + ground; matches openmv_dual_color_uart.py)
//   OpenMV P4 (UART1 TX) -> Teensy pin 28 (RX7)
//   OpenMV P5 (UART1 RX) -> Teensy pin 29 (TX7)   [optional, unused]
//   OpenMV GND           -> Teensy GND
//
// Both ends must use the same baud rate (115200 below).
//
// PACKET FORMAT (8 bytes, matches the OpenMV sender)
//   byte 0: sync byte   - 0xAA = color A, 0xAB = color B, 0xAC = color C
//   byte 1: detected    - 0 or 1
//   byte 2-3: angle_deg * 100, signed 16-bit, big-endian
//   byte 4-5: radius_px, unsigned 16-bit, big-endian
//   byte 6: size_byte    - pixel_count / 4, clamped to 0-255
//   byte 7: checksum     - XOR of bytes 0-6
//
// Since UART has no chip-select to frame a transfer the way SPI does,
// this receiver scans the incoming byte stream for a valid sync byte,
// then reads the following 7 bytes as one packet and validates the
// checksum before accepting it. Any stray/misaligned bytes are
// discarded and the parser re-syncs on the next valid sync byte.

#define TEENSY_UART Serial7
#define UART_BAUD 115200

const uint8_t PACKET_LEN = 8;
const uint8_t SYNC_A = 0xAA;
const uint8_t SYNC_B = 0xAB;
const uint8_t SYNC_C = 0xAC;

struct BallPacket {
  bool detected = false;
  float angle_deg = 0.0f;
  float radius_px = 0.0f;
  uint8_t size_byte = 0;
  uint32_t last_update_ms = 0;
};

BallPacket ballA, ballB, ballC;

uint8_t packetBuf[PACKET_LEN];
uint8_t packetIdx = 0;
bool syncFound = false;

bool isSyncByte(uint8_t b) {
  return (b == SYNC_A || b == SYNC_B || b == SYNC_C);
}

uint8_t computeChecksum(const uint8_t *p) {
  uint8_t c = 0;
  for (uint8_t i = 0; i < PACKET_LEN - 1; i++) {
    c ^= p[i];
  }
  return c;
}

void decodePacket(const uint8_t *p) {
  uint8_t sync = p[0];
  bool detected = p[1] != 0;
  int16_t angle_x100 = (int16_t)((p[2] << 8) | p[3]);
  uint16_t radius_i = (uint16_t)((p[4] << 8) | p[5]);
  uint8_t size_byte = p[6];

  BallPacket result;
  result.detected = detected;
  result.angle_deg = angle_x100 / 100.0f;
  result.radius_px = (float)radius_i;
  result.size_byte = size_byte;
  result.last_update_ms = millis();

  switch (sync) {
    case SYNC_A: ballA = result; break;
    case SYNC_B: ballB = result; break;
    case SYNC_C: ballC = result; break;
  }
}

// Call this often (e.g. every loop()) to drain the UART RX buffer and
// decode any complete, checksum-valid packets it contains.
void pollUart() {
  while (TEENSY_UART.available()) {
    uint8_t b = TEENSY_UART.read();

    if (!syncFound) {
      // Discard bytes until we see a valid sync byte, then start
      // capturing a new packet.
      if (isSyncByte(b)) {
        packetBuf[0] = b;
        packetIdx = 1;
        syncFound = true;
      }
      continue;
    }

    packetBuf[packetIdx++] = b;

    if (packetIdx == PACKET_LEN) {
      if (computeChecksum(packetBuf) == packetBuf[PACKET_LEN - 1]) {
        decodePacket(packetBuf);
      }
      // Checksum mismatch -> silently drop this packet and resync on
      // the next sync byte seen.
      syncFound = false;
      packetIdx = 0;
    }
  }
}

void setup() {
  Serial.begin(115200);       // USB debug console
  TEENSY_UART.begin(UART_BAUD);
}

void loop() {
  pollUart();

  static uint32_t lastPrint = 0;
  if (millis() - lastPrint > 200) {
    lastPrint = millis();
    Serial.printf(
      "A: det=%d ang=%.2f rad=%.1f sz=%u | "
      "B: det=%d ang=%.2f rad=%.1f sz=%u | "
      "C: det=%d ang=%.2f rad=%.1f sz=%u\n",
      ballA.detected, ballA.angle_deg, ballA.radius_px, ballA.size_byte,
      ballB.detected, ballB.angle_deg, ballB.radius_px, ballB.size_byte,
      ballC.detected, ballC.angle_deg, ballC.radius_px, ballC.size_byte
    );
  }
}
