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
  initDisplay();
  initBallTracking();
  initLocalization();
  initCommunication();

  initDribbler();
  setDribblerDirectionReverse();

  analogReadResolution(ADC_RESOLUTION_BITS);

  lastBatteryCheckMs = millis();
  checkBattery();

  initIMU();

  Serial.print("Initial Heading: ");
  Serial.println(currentYawDeg);
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
