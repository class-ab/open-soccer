/*
  Merged robot sketch (Teensy 4.1)
  =================================
  Full robot control code (motors, IMU heading hold, OLED status display,
  battery protection, ball-chase) split across subsystems under src/subsystems/.
*/

#include <Arduino.h>
#include <Wire.h>

#include "include/robot.h"

#include "include/subsystems/vision.h"
#include "include/subsystems/battery.h"
#include "include/subsystems/display.h"
#include "include/subsystems/drivebase.h"
#include "include/subsystems/dribbler.h"
#include "include/subsystems/imu.h"
#include "include/subsystems/robot_config.h"
#include "include/subsystems/robot_state.h"
#include "include/subsystems/robot_tick.h"

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
  systemTick();

  if (shutdownLatched) {
    return;
  }

  updateIMU();

  chaseTick();   

  setDribblerThrottle(1200);

  if(!robotCurrentlyRunning) {
    stopAllMotors();
  }

}

void systemTick() {
  unsigned long now = millis();

  checkEnabledButton(now);

  processBallPacket();

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
static unsigned long lastDebounceMs = 0;
const unsigned long BUTTON_DEBOUNCE_MS = 50;

bool currentButton1State = digitalRead(button1);

// If the reading changed, reset the debounce timer
if (currentButton1State != lastButton1State) {
lastDebounceMs = now;
}

// If enough time has passed since the last change and the state is stable...
if ((now - lastDebounceMs) >= BUTTON_DEBOUNCE_MS) {
  // Detect rising edge (LOW->HIGH).
  static bool stableLastState = LOW;
  if (currentButton1State == HIGH && stableLastState == LOW) {
  robotCurrentlyRunning = !robotCurrentlyRunning;
  lastRunStateChangeMs = now;
  Serial.println(robotCurrentlyRunning ? "Robot RUNNING" : "Robot STOPPED");
  }
  stableLastState = currentButton1State;
  updateDisplay();
}
lastButton1State = currentButton1State;
}

void stopAllMotors() {
  stopAllDriveMotors();
  stopDribbler();
}
