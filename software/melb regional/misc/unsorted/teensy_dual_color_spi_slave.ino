// Teensy 4.1 - Hardware SPI slave receiver for the OpenMV RT1062 dual-color
// blob tracker (dual_color_blob_tracking_spi.py).
//
// Uses the SPISlave_T4 library (https://github.com/tonton81/SPISlave_T4)
// to run the Teensy's real LPSPI peripheral in slave mode, framed by CS -
// no bit-banged GPIO clock decoding, no manual interrupt-driven bit
// shifting. Install SPISlave_T4 via Library Manager / Arduino Library
// Manager or by cloning it into your Arduino libraries folder.
//
// WIRING (matches the OpenMV script's header comment):
//   OpenMV P0 (SPI1 SCLK) -> Teensy pin 13 (SCK)
//   OpenMV P1 (SPI1 MOSI) -> Teensy pin 11 (MOSI)
//   OpenMV P2 (SPI1 MISO) -> Teensy pin 12 (MISO)  [unused]
//   OpenMV P3 (SPI1 CS)   -> Teensy pin 10 (CS)
//   OpenMV GND            -> Teensy GND
//
// PACKET FORMAT (8 bytes, MSB-first, mode 0, one CS pulse per packet):
//   [0] sync byte   0xAA = color A, 0xAB = color B
//   [1] detected    0 or 1
//   [2:3] angle_x100  int16, big-endian, degrees * 100
//   [4:5] radius      uint16, big-endian, pixels
//   [6] size_byte     pixel_count / 4, clamped to 0-255
//   [7] checksum      XOR of bytes [0:7)

#include <SPISlave_T4.h>

SPISlave_T4<&SPI, SPI_8_BITS> spiSlave;

#define PACKET_SYNC_BYTE_A 0xAA
#define PACKET_SYNC_BYTE_B 0xAB
#define PACKET_LEN 8

// Latest decoded state for each tracked color. Written from the SPI ISR,
// read from loop() - mark volatile since they cross that boundary.
volatile bool colorADetected = false;
volatile float colorAAngleDeg = 0.0f;
volatile uint16_t colorARadiusPx = 0;
volatile uint8_t colorASizeByte = 0;

volatile bool colorBDetected = false;
volatile float colorBAngleDeg = 0.0f;
volatile uint16_t colorBRadiusPx = 0;
volatile uint8_t colorBSizeByte = 0;

volatile uint32_t framingErrors = 0;
volatile uint32_t checksumErrors = 0;

// Per-packet receive buffer, filled a byte at a time inside the ISR.
static uint8_t rxBuf[PACKET_LEN];
static uint8_t rxIndex = 0;

void handlePacket(const uint8_t *packet) {
  uint8_t checksum = 0;
  for (uint8_t i = 0; i < 7; i++) {
    checksum ^= packet[i];
  }
  if (checksum != packet[7]) {
    checksumErrors++;
    return;
  }

  uint8_t sync = packet[0];
  bool detected = packet[1] != 0;
  int16_t angleX100 = (int16_t)((packet[2] << 8) | packet[3]);
  uint16_t radiusPx = (uint16_t)((packet[4] << 8) | packet[5]);
  uint8_t sizeByte = packet[6];
  float angleDeg = angleX100 / 100.0f;

  if (sync == PACKET_SYNC_BYTE_A) {
    colorADetected = detected;
    colorAAngleDeg = angleDeg;
    colorARadiusPx = radiusPx;
    colorASizeByte = sizeByte;
  } else if (sync == PACKET_SYNC_BYTE_B) {
    colorBDetected = detected;
    colorBAngleDeg = angleDeg;
    colorBRadiusPx = radiusPx;
    colorBSizeByte = sizeByte;
  } else {
    framingErrors++;
  }
}

// Called by the SPISlave_T4 ISR whenever CS is asserted and bytes arrive.
// Each CS pulse from the OpenMV cam carries exactly one 8-byte packet.
void onSpiReceive() {
  rxIndex = 0;
  while (spiSlave.active()) {
    if (spiSlave.available()) {
      uint8_t b = (uint8_t)spiSlave.popr();
      if (rxIndex < PACKET_LEN) {
        rxBuf[rxIndex++] = b;
      } else {
        rxIndex++;  // keep counting so overrun is visible in the debug print
      }
    }
  }

  if (rxIndex == PACKET_LEN) {
    handlePacket(rxBuf);
  } else {
    framingErrors++;
    // DEBUG: print exactly what we captured. Remove once framing is fixed.
    Serial.print("framing err, got ");
    Serial.print(rxIndex);
    Serial.print(" bytes: ");
    for (uint8_t i = 0; i < rxIndex && i < PACKET_LEN; i++) {
      if (rxBuf[i] < 0x10) Serial.print('0');
      Serial.print(rxBuf[i], HEX);
      Serial.print(' ');
    }
    Serial.println();
  }
}

void setup() {
  Serial.begin(115200);
  spiSlave.onReceive(onSpiReceive);
  spiSlave.begin();
}

void loop() {
  // Example: print the latest decoded state for both balls a few times a
  // second. Replace this with whatever your robot logic needs - the
  // colorA*/colorB* variables above always hold the most recent packet.
  static uint32_t lastPrint = 0;
  if (millis() - lastPrint >= 200) {
    lastPrint = millis();

    noInterrupts();
    bool aDet = colorADetected;
    float aAng = colorAAngleDeg;
    uint16_t aRad = colorARadiusPx;
    uint8_t aSize = colorASizeByte;
    bool bDet = colorBDetected;
    float bAng = colorBAngleDeg;
    uint16_t bRad = colorBRadiusPx;
    uint8_t bSize = colorBSizeByte;
    uint32_t frErr = framingErrors;
    uint32_t chkErr = checksumErrors;
    interrupts();

    Serial.printf(
        "A: det=%d angle=%.2f radius=%u size=%u | "
        "B: det=%d angle=%.2f radius=%u size=%u | "
        "framingErr=%lu checksumErr=%lu\n",
        aDet, aAng, aRad, aSize,
        bDet, bAng, bRad, bSize,
        (unsigned long)frErr, (unsigned long)chkErr);
  }
}
