#ifndef SUBSYSTEMS_ROBOT_STATE_H
#define SUBSYSTEMS_ROBOT_STATE_H

#include <Arduino.h>

// ============================================================
// Uptime / Run-Idle Timers
// ============================================================
// Defined and updated in robot.ino. Read by display.cpp (status screen)
// and battery.cpp (shutdown-screen uptime readout).

// millis() captured once in setup().
extern unsigned long bootMillis;

// millis() of the last run<->idle transition.
extern unsigned long lastRunStateChangeMs;

// True for the duration of a move() sequence (e.g. the button1 square
// demo). Read by display.cpp to show RUNNING/STOPPED.
extern bool robotCurrentlyRunning;

// ============================================================
// systemTick
// ============================================================
// Defined in robot.ino. Non-blocking scheduler for the periodic
// background jobs: draining any newly-completed ball packet, the 5s
// battery check (which can trip emergencyShutdown()), and the display
// refresh. Declared here so subsystem .cpp files that block for a while
// internally - namely drivebase's move() - can call it from inside their
// own loop, catching a battery cutoff or a fresh ball packet within one
// tick instead of waiting for the whole call to finish.
void systemTick();

#endif
