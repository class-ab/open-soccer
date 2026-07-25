// teensy_dual_color_bitbang_slave.ino
const int SCK_PIN  = 13;
const int MOSI_PIN = 11;
const int MISO_PIN = 12;
const int CS_PIN   = 10;

const uint8_t PACKET_LEN = 8;
const uint8_t SYNC_A = 0xAA;
const uint8_t SYNC_B = 0xAB;

volatile uint8_t rxByte    = 0;
volatile uint8_t bitCount  = 0;
volatile uint8_t byteIndex = 0;
volatile uint8_t rxBuf[PACKET_LEN];

volatile uint8_t readyBuf[PACKET_LEN];
volatile bool packetReady = false;

void onClockEdge() {
  rxByte <<= 1;
  if (digitalReadFast(MOSI_PIN)) rxByte |= 1;
  bitCount++;
  if (bitCount == 8) {
    if (byteIndex < PACKET_LEN) {
      rxBuf[byteIndex] = rxByte;
      byteIndex++;
    }
    bitCount = 0;
    rxByte = 0;
  }
}

// Single handler for BOTH edges of CS.
void onCSChange() {
  if (digitalReadFast(CS_PIN) == LOW) {
    // Falling edge: start of a new framed transfer.
    bitCount = 0;
    byteIndex = 0;
    rxByte = 0;
  } else {
    // Rising edge: transfer done. Only accept a full 8-byte packet.
    if (byteIndex == PACKET_LEN) {
      for (uint8_t i = 0; i < PACKET_LEN; i++) readyBuf[i] = rxBuf[i];
      packetReady = true;
    }
  }
}

void setup() {
  Serial.begin(115200);
  pinMode(SCK_PIN, INPUT);
  pinMode(MOSI_PIN, INPUT);
  pinMode(MISO_PIN, OUTPUT);
  pinMode(CS_PIN, INPUT);
  Serial.println("START");

  attachInterrupt(digitalPinToInterrupt(CS_PIN), onCSChange, CHANGE);
  attachInterrupt(digitalPinToInterrupt(SCK_PIN), onClockEdge, RISING);
}

void loop() {
  onCSChange();
  if (packetReady) {
    packetReady = false;

    uint8_t buf[PACKET_LEN];
    noInterrupts();
    for (uint8_t i = 0; i < PACKET_LEN; i++) buf[i] = readyBuf[i];
    interrupts();

    uint8_t checksum = 0;
    for (uint8_t i = 0; i < PACKET_LEN - 1; i++) checksum ^= buf[i];
    if (checksum != buf[PACKET_LEN - 1]) return;  // corrupted packet, drop

    uint8_t sync = buf[0];
    if (sync != SYNC_A && sync != SYNC_B) return;

    bool detected       = buf[1];
    int16_t angle_x100  = (int16_t)((buf[2] << 8) | buf[3]);
    uint16_t radius     = (uint16_t)((buf[4] << 8) | buf[5]);
    uint8_t size_byte   = buf[6];

    const char *label = (sync == SYNC_A) ? "A" : "B";
    Serial.printf("%s  det=%d  angle=%.2f  radius=%u  size=%u\n",
                  label, detected, angle_x100 / 100.0, radius, size_byte);
  }
}