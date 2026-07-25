/*
  Merged robot sketch (Teensy 4.1)
  =================================
  Full robot control code (motors, IMU heading hold, OLED status display,
  battery protection, ball-chase) split across subsystems under src/subsystems/.
*/

#include <Arduino.h>
#include <Wire.h>

#include "subsystems/vision.h"
#include "subsystems/battery.h"
#include "subsystems/display.h"
#include "subsystems/drivebase.h"
#include "subsystems/imu.h"
#include "subsystems/robot_config.h"
#include "subsystems/robot_state.h"
#include "subsystems/robot_tick.h"

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

  analogReadResolution(ADC_RESOLUTION_BITS);

  lastBatteryCheckMs = millis();
  checkBattery();

  initIMU();

  Serial.print("Initial Heading: ");
  Serial.println(currentYawDeg);
}

void loop() {
  systemTick();

  if (shutdownLatched) {
    return;
  }

  updateIMU();

  if (digitalRead(button1) == HIGH) {
    delay(100);

    updateIMU();

    float startHeading = currentYawDeg;

    Serial.print("Starting Heading: ");
    Serial.println(startHeading);

    robotCurrentlyRunning = true;
    lastRunStateChangeMs = millis();

    move(startHeading + 0,   1500, startHeading + 90.0f,  ROBOT_MAX_SPEED);
    move(startHeading + 90,  1500, startHeading + 180.0f, ROBOT_MAX_SPEED);
    move(startHeading + 180, 1500, startHeading + 270.0f, ROBOT_MAX_SPEED);
    move(startHeading + 270, 1500, startHeading,          ROBOT_MAX_SPEED);

    stopAllMotors();

    robotCurrentlyRunning = false;
    lastRunStateChangeMs = millis();
  }

  chaseTick();
}

void systemTick() {
  unsigned long now = millis();

  processBallPacket();

  if (now - lastBatteryCheckMs >= BATTERY_CHECK_INTERVAL_MS) {
    checkBattery();
  }

  if (now - lastDisplayUpdateMs >= DISPLAY_UPDATE_INTERVAL_MS) {
    lastDisplayUpdateMs = now;
    updateDisplay();
  }
}
