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
#include "include/subsystems/attack.h"
#include "include/subsystems/defend.h"

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
  Wire2.setClock(400000);
  initDisplay();
  initBallTracking();

  initDribbler();
  setDribblerDirectionForward();

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

  // When attacking but we have not seen the ball for BALL_LOST_RETURN_HOME_MS,
  // fall back to defending (return home and face the ball) until it reappears.
  bool ballSeenRecently = (millis() - lastBallSeenMs) <= BALL_LOST_RETURN_HOME_MS;
  if (isAttacking && ballSeenRecently) {
    attackTick();
  } else {
    defendTick();
  }

  if (robotCurrentlyRunning && dribblerShouldRun) {
    setDribblerThrottle(DRIBBLER_RUN_THROTTLE_US);
  } else {
    stopDribbler();
  }

  if(!robotCurrentlyRunning) {
    stopAllMotors();
  }

}

void systemTick() {
  unsigned long now = millis();

  checkEnabledButton(now);
  checkAllianceButtons(now);
  updateHasBall();

  processVisionPackets();

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

void checkAllianceButtons(unsigned long now) {
  // Button 2 -> toggle between yellow and blue alliance.
  // Button 3 -> toggle between attacking and defending.
  // Same debounced rising-edge detection as checkEnabledButton, with one
  // debounce state per button.

  // ---------- button2 (toggle alliance) ----------
  static bool lastButton2State = LOW;
  static unsigned long lastButton2DebounceMs = 0;
  const unsigned long BUTTON_DEBOUNCE_MS = 50;

  bool currentButton2State = digitalRead(button2);
  if (currentButton2State != lastButton2State) {
    lastButton2DebounceMs = now;
  }
  if ((now - lastButton2DebounceMs) >= BUTTON_DEBOUNCE_MS) {
    static bool stableButton2State = LOW;
    if (currentButton2State == HIGH && stableButton2State == LOW) {
      isYellowAlliance = !isYellowAlliance;
      Serial.println(isYellowAlliance ? "Alliance: YELLOW" : "Alliance: BLUE");
    }
    stableButton2State = currentButton2State;
  }
  lastButton2State = currentButton2State;

  // ---------- button3 (toggle attack/defend) ----------
  static bool lastButton3State = LOW;
  static unsigned long lastButton3DebounceMs = 0;

  bool currentButton3State = digitalRead(button3);
  if (currentButton3State != lastButton3State) {
    lastButton3DebounceMs = now;
  }
  if ((now - lastButton3DebounceMs) >= BUTTON_DEBOUNCE_MS) {
    static bool stableButton3State = LOW;
    if (currentButton3State == HIGH && stableButton3State == LOW) {
      isAttacking = !isAttacking;
      Serial.println(isAttacking ? "Mode: ATTACK" : "Mode: DEFEND");
    }
    stableButton3State = currentButton3State;
  }
  lastButton3State = currentButton3State;
}

void updateHasBall() {
  // "Has ball" is true when the latest ball packet reports the ball
  // detected, within a reasonable range, and roughly in front of the
  // robot (angleDeg is relative to the robot front/dribbler direction,
  // 0 = straight ahead). Uses fabsf on the packet field.
  hasBall = latestBallPacket.detected
            && latestBallPacket.distanceCM <= HAS_BALL_MAX_DISTANCE_CM
            && fabsf(latestBallPacket.angleDeg) <= HAS_BALL_MAX_ANGLE_DEG;
}

void stopAllMotors() {
  stopAllDriveMotors();
  stopDribbler();
}
