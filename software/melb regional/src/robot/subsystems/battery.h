#ifndef SUBSYSTEMS_BATTERY_H
#define SUBSYSTEMS_BATTERY_H

#include <Arduino.h>

// Latches true the instant a low-battery shutdown fires. Once set, the
// robot is intentionally dead until power-cycled - see the comment above
// emergencyShutdown()'s definition in battery.cpp for why this isn't
// auto-resumable. Checked by robot.ino's loop() and by display.cpp's
// updateDisplay().
extern bool shutdownLatched;

// Most recent battery reading, volts. Updated by checkBattery(); read by
// display.cpp for the status screen.
extern float lastBatteryVoltage;

// Configures ADC resolution and takes an immediate first reading. Call
// once from setup(), before the IMU/motors are touched, so a
// dead/miswired/already-too-low battery is caught before anything else
// spins up. May call emergencyShutdown() (and never return) if the
// battery is already below the shutdown threshold.
void initBattery();

// Reads the battery, stores it (lastBatteryVoltage) for the display, and
// immediately trips emergencyShutdown() if it's below
// BATTERY_SHUTDOWN_VOLTAGE.
void checkBattery();

// Non-blocking scheduler - calls checkBattery() no more often than every
// BATTERY_CHECK_INTERVAL_MS. Call this every tick; systemTick() does.
void batteryTick();

// Immediately halts the robot: stops every motor, latches shutdownLatched,
// shows the permanent warning screen, then hangs forever. Not
// auto-resumable by design - see the comment above its definition in
// battery.cpp.
void emergencyShutdown();

#endif
