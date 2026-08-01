/*
  Merged robot sketch (Teensy 4.1)
  =================================
  Full robot control code (motors, IMU heading hold, OLED status display,
  battery protection, ball-chase) split across subsystems under src/subsystems/.
*/

#include <Arduino.h>
#include <Wire.h>

#include "include/subsystems/vision.h"
#include "include/subsystems/battery.h"
#include "include/subsystems/display.h"
#include "include/subsystems/drivebase.h"
#include "include/subsystems/imu.h"
#include "include/subsystems/robot_config.h"
#include "include/subsystems/robot_state.h"
#include "include/subsystems/robot_tick.h"

unsigned long now;

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

  chaseTick();
}

void systemTick() {
  now = millis();

  processBallPacket();

  if (now - lastBatteryCheckMs >= BATTERY_CHECK_INTERVAL_MS) {
    checkBattery();
  }

  if (now - lastDisplayUpdateMs >= DISPLAY_UPDATE_INTERVAL_MS) {
    lastDisplayUpdateMs = now;
    updateDisplay();
  }
}

void checkEnabledButton() {
  static bool lastButton1State = LOW;
  bool currentButton1State = digitalRead(button1);

  if (currentButton1State == HIGH && lastButton1State == LOW) {
    if (!robotCurrentlyRunning) {
      robotCurrentlyRunning = true;
      lastRunStateChangeMs = now;
      Serial.println("Robot RUNNING");
    }
    else {
      robotCurrentlyRunning = false;
      lastRunStateChangeMs = now;
      Serial.println("Robot STOPPED");
    }
    lastButton1State = HIGH;
  }

  if(currentButton1State == LOW) {
    lastButton1State = LOW;
  }
}
