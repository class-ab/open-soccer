#include <Arduino.h>
#include <Wire.h>

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
#include "include/subsystems/robot_tick.h"
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
  Serial.println("BOOT 1/6: display");
  initDisplay();
  showBootStatus("BOOT 2/6", "Ball UART");
  initBallTracking();
  showBootStatus("BOOT 3/6", "LiDAR UART");
  initLocalization();
  showBootStatus("BOOT 4/6", "RF24 radio");
  if (!initCommunication()) {
    haltBoot("RF24 NOT FOUND", "Check SPI, CE/CSN");
  }

  showBootStatus("BOOT 5/6", "Dribbler ESC");
  initDribbler();
  setDribblerDirectionReverse();

  analogReadResolution(ADC_RESOLUTION_BITS);

  lastBatteryCheckMs = millis();
  checkBattery();

  showBootStatus("BOOT 6/6", "BNO08x IMU");
  if (!initIMU()) {
    haltBoot("BNO08X NOT FOUND", "Check I2C/power");
  }

  Serial.print("Initial Heading: ");
  Serial.println(currentYawDeg);
  Serial.println("BOOT COMPLETE");
}

void loop() {
  // main update system
  systemTick();
  // check if battery under-voltage shutdown has been latched
  if (shutdownLatched) {
    return;
  }

  updateStrategy();

  if(!robotCurrentlyRunning) {
    stopAllMotors();
  }

  /* OLD CODE
  chaseBall();
  if (robotCurrentlyRunning && dribblerShouldRun) {
    setDribblerThrottle(DRIBBLER_RUN_THROTTLE_US);
  } else {
    stopDribbler();
  }
  */
}

void systemTick() {
  unsigned long now = millis();
  checkButtons();
  updateIMU();
  checkEnabledButton(now);
  processBallPacket();
  updateLocalization();
  updateCommunication();

  if (now - lastBatteryCheckMs >= BATTERY_CHECK_INTERVAL_MS) {
    checkBattery();
  }

  if (now - lastDisplayUpdateMs >= DISPLAY_UPDATE_INTERVAL_MS) {
    lastDisplayUpdateMs = now;
    updateDisplay();
  }
}

void checkEnabledButton(unsigned long now) {
  static bool lastButton1State = LOW;

  if (button1State == HIGH && lastButton1State == LOW) {
  robotCurrentlyRunning = !robotCurrentlyRunning;
  updateLocalRobotMode();
  lastRunStateChangeMs = now;
  Serial.println(robotCurrentlyRunning ? "Robot RUNNING" : "Robot STOPPED");
  updateDisplay();
  }

  lastButton1State = button1State;

  static bool lastButton3State = LOW;
  if (button3State == HIGH && lastButton3State == LOW) {
    updateLocalRobotMode();
  }

  lastButton3State = button3State;
}

void stopAllMotors() {
  stopAllDriveMotors();
  stopDribbler();
}
