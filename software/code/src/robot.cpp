#include <Arduino.h>
#include <Wire.h>
#include <math.h>

#include "include/robot.h"
#include "include/subsystems/vision.h"
#include "include/subsystems/battery.h"
#include "include/subsystems/communication.h"
#include "include/subsystems/display.h"
#include "include/subsystems/drivebase.h"
#include "include/subsystems/dribbler.h"
#include "include/subsystems/imu.h"
#include "include/subsystems/localization.h"
#include "include/subsystems/robot_config.h"
#include "include/subsystems/robot_state.h"
#include "include/subsystems/strategy.h"

namespace {
void haltBoot(const char *device, const char *check) {
  Serial.print("BOOT FAILED: ");
  Serial.println(device);
  showBootStatus("BOOT FAILED", device);
  delay(1200);
  showBootStatus(device, check);

  // Do not enter loop(): it would try to use subsystems that did not start.
  // Keep every output stopped while the error remains visible on the OLED.
  stopAllDriveMotors();
  stopDribbler();
  while (true) {
    delay(250);
  }
}
}

void setup() {
  Serial.begin(115200);

  pinMode(M1a, OUTPUT);
  pinMode(M1b, OUTPUT);
  pinMode(M2a, OUTPUT);
  pinMode(M2b, OUTPUT);
  pinMode(M3a, OUTPUT);
  pinMode(M3b, OUTPUT);
  pinMode(M4a, OUTPUT);
  pinMode(M4b, OUTPUT);

  pinMode(button1, INPUT);
  pinMode(button2, INPUT);
  pinMode(button3, INPUT);

  delay(100);

  Serial.println("Starting...");

  bootMillis = millis();
  lastRunStateChangeMs = bootMillis;

  Wire2.begin();
  Serial.println("BOOT 1/7: display");
  initDisplay();
  showBootStatus("BOOT 2/7", "Ball UART");
  initBallTracking();
  showBootStatus("BOOT 3/7", "LiDAR UART");
  initLocalization();
  showBootStatus("BOOT 4/7", "RF24 radio");
  if (!initCommunication()) {
    haltBoot("RF24 NOT FOUND", "Check SPI, CE/CSN");
  }

  showBootStatus("BOOT 5/7", "Dribbler ESC");
  initDribbler();
  setDribblerDirectionReverse();

  analogReadResolution(ADC_RESOLUTION_BITS);

  lastBatteryCheckMs = millis();
  checkBattery();

  showBootStatus("BOOT 6/7", "BNO08x IMU");
  if (!initIMU()) {
    haltBoot("BNO08X NOT FOUND", "Check I2C/power");
  }
  initKicker();

  Serial.print("Initial Heading: ");
  Serial.println(currentYawDeg);
  Serial.println("BOOT COMPLETE");
}

void loop() {
  // main update system
  unsigned long now = millis();
  checkButtons();
  updateIMU();
  processBallPacket();
  updateLocalization();
  updateCommunication();
  updateStrategy();
  updateKicker();

  // Refresh after localization and strategy work, just before control uses it.
  updateIMU();
  
  if (robotCurrentlyRunning) {
    move();
  } else {
    stopAllMotors();
  }

  if (now - lastBatteryCheckMs >= BATTERY_CHECK_INTERVAL_MS) {
    checkBattery(); // check battery at time interval
  }

  if (now - lastDisplayUpdateMs >= DISPLAY_UPDATE_INTERVAL_MS) {
    lastDisplayUpdateMs = now;
    updateDisplay(); // update display at time interval
  }

  if (shutdownLatched) {
    return; // undervoltage check
  }
}

void stopAllMotors() {
  stopAllDriveMotors();
  stopDribbler();
}
