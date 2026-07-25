/*
  robot.ino - Main sketch (Teensy 4.1)
  =====================================
  Full robot control (motors, IMU heading hold, OLED status display,
  battery protection, ball-chase over a CS-framed bit-banged SPI-style
  link to an OpenMV cam) split into one file per subsystem under
  subsystems/:

    subsystems/imu.h           - BNO08x heading (currentYawDeg)
    subsystems/drivebase.h     - motors, field-oriented drive(), move(),
                                  heading-hold PID
    subsystems/display.h       - OLED status screen
    subsystems/battery.h       - battery monitor + emergency shutdown
    subsystems/ball_tracking.h - link to the OpenMV cam + chaseTick()
    subsystems/robot_state.h   - shared robot-wide state + systemTick()
    subsystems/robot_config.h  - build-time DEBUG_* toggles

  This file just wires them together: setup(), loop(), the periodic
  systemTick() housekeeping call, and the two button-triggered
  behaviors (square demo path, ball chase). See each header above for
  what its subsystem owns, and each .cpp for the implementation detail
  and wiring/tuning notes that used to live in this single file.
*/

#include <Arduino.h>
#include <Wire.h>

#include "subsystems/robot_config.h"
#include "subsystems/robot_state.h"
#include "subsystems/imu.h"
#include "subsystems/drivebase.h"
#include "subsystems/display.h"
#include "subsystems/battery.h"
#include "subsystems/ball_tracking.h"

// ============================================================
// Buttons
// ============================================================

const int button1 = A6; // press: run the demo square sequence
const int button2 = A7; // press: chase the ball (see loop())
const int button3 = A8;

// ============================================================
// Uptime / Run-Idle Timers
// ============================================================
// Declared extern in subsystems/robot_state.h; defined here since
// robot.ino owns setting them (button1's square-demo handler below).

unsigned long bootMillis = 0;
unsigned long lastRunStateChangeMs = 0;
bool robotCurrentlyRunning = false;

// ============================================================
// Setup
// ============================================================

void setup() {
  Serial.begin(115200);

  initDrivebase();

  pinMode(button1, INPUT);
  pinMode(button2, INPUT);
  pinMode(button3, INPUT);

  delay(100);

  Serial.println("Starting...");

  bootMillis = millis();
  lastRunStateChangeMs = bootMillis;

  Wire2.begin();
  initDisplay();

  // Ball-tracking receiver (CS-framed, bit-banged).
  initBallTracking();

  // Takes an immediate battery reading before anything else spins up;
  // may call emergencyShutdown() (and never return) if the battery is
  // already too low.
  initBattery();

  // Hangs forever if the BNO08x isn't found - the robot isn't safe to
  // drive without a working IMU.
  initIMU();
}

// ============================================================
// Main Loop
// ============================================================

void loop() {
  systemTick(); // periodic battery check + display refresh + ball-packet drain

  if (shutdownLatched) {
    // emergencyShutdown() already hangs forever internally, but this
    // guard is here too in case loop() is ever re-entered after a
    // shutdown for any reason - nothing below this point may run again.
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

    // Example sequence: a 1.5s-per-side square, each side also gradually
    // rotating a quarter turn, unwinding back to the start heading on the
    // final leg.
    //
    // direction_deg is FIELD-relative (see drivebase.cpp's drive()), so
    // each side's direction is offset by startHeading - this anchors
    // "field forward" to whichever way the robot happened to be facing
    // at button press, the same reference the rotation targets below
    // use. The robot will trace a straight-sided square in that fixed
    // frame even though it's spinning a quarter turn on every side.
    move(startHeading + 0,   1500, startHeading + 90.0f,  0.7f);
    move(startHeading + 90,  1500, startHeading + 180.0f, 0.7f);
    move(startHeading + 180, 1500, startHeading + 270.0f, 0.7f);
    move(startHeading + 270, 1500, startHeading,          0.7f);

    stopAllMotors();

    robotCurrentlyRunning = false;
    lastRunStateChangeMs = millis();
  }

  // Chases the ball using the latest bearing/radius data from the
  // OpenMV cam (see subsystems/ball_tracking.cpp's chaseTick()). A dead
  // battery stops it immediately (via emergencyShutdown()); losing the
  // ball for longer than BALL_DATA_TIMEOUT_MS just makes chaseTick()
  // hold heading and stop translating until the ball is seen again.
  chaseTick();
}

// ============================================================
// systemTick - periodic background jobs
// ============================================================
// Non-blocking scheduler for the background jobs: draining any
// newly-completed ball packet, the periodic battery check (which can
// trip emergencyShutdown()), and the display refresh. Call this from
// loop() AND from inside move()'s while loop (drivebase.cpp does), so a
// battery cutoff or a fresh ball packet is caught within one tick even
// mid-move rather than waiting for the current move to finish.

void systemTick() {
  processBallPacket();
  batteryTick();
  displayTick();
}
