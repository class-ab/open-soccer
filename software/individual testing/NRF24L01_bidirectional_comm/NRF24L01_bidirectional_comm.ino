#include <SPI.h>
#include <RF24.h>

#define CE_PIN  3
#define CSN_PIN 2

RF24 radio(CE_PIN, CSN_PIN);

// Two pipes:
// Pipe A: Node 1 -> Node 2
// Pipe B: Node 2 -> Node 1
const byte pipeA[6] = "NODE1";
const byte pipeB[6] = "NODE2";

// Change this on ONE board only:
// Board 1 = true
// Board 2 = false
const bool IS_NODE1 = false;

char txBuffer[32];
char rxBuffer[32];

void setup() {
  Serial.begin(115200);

  while (!Serial) {}

  if (!radio.begin()) {
    Serial.println("nRF24L01 not detected!");
    while (1);
  }

  radio.setPALevel(RF24_PA_LOW);
  radio.setDataRate(RF24_250KBPS);
  radio.setChannel(76);

  radio.enableDynamicPayloads();

  if (IS_NODE1) {
    radio.openWritingPipe(pipeA);
    radio.openReadingPipe(1, pipeB);
  } else {
    radio.openWritingPipe(pipeB);
    radio.openReadingPipe(1, pipeA);
  }

  radio.startListening();

  Serial.println("Ready.");
  Serial.println("Type text and press Enter.");
}

void loop() {

  // Check for incoming radio messages
  if (radio.available()) {

    uint8_t len = radio.getDynamicPayloadSize();

    if (len > sizeof(rxBuffer) - 1) {
      len = sizeof(rxBuffer) - 1;
    }

    radio.read(rxBuffer, len);
    rxBuffer[len] = '\0';

    Serial.print("RX: ");
    Serial.println(rxBuffer);
  }

  // Check for serial input
  if (Serial.available()) {

    String msg = Serial.readStringUntil('\n');
    msg.trim();

    if (msg.length() > 0) {

      msg.toCharArray(txBuffer, sizeof(txBuffer));

      radio.stopListening();

      bool ok = radio.write(
        txBuffer,
        strlen(txBuffer) + 1
      );

      radio.startListening();

      if (ok) {
        Serial.print("TX: ");
        Serial.println(txBuffer);
      } else {
        Serial.println("Send failed");
      }
    }
  }
}