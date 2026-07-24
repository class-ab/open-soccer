/*
 Name:		Teensy_SPIder.ino
 Created:	5/5/2023 12:21:04 PM
 Author:	tj
*/

uint32_t spiRx[10];
volatile int spiRxIdx;
volatile int spiRxComplete = 0;

#include "SPISlave_T4.h"
SPISlave_T4<&SPI, SPI_8_BITS> mySPI;

void setup() {
  Serial.begin(115200);	//baudrate does not matter (is USB VCP anyway)
  while ( ! Serial) {}
  Serial.println("START...");
  mySPI.begin();
}

void loop() {
  int i;

  Serial.print("millis: "); Serial.println(millis());
  if (spiRxComplete) {
    Serial.println(spiRxIdx);
    for (i = 0; i < spiRxIdx; i++) {
      Serial.print(spiRx[i], HEX); Serial.print(" ");
    }
    Serial.println();
    spiRxComplete = 0;
    spiRxIdx = 0;
  }
  delay(1000);
}

