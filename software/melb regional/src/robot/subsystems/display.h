#ifndef SUBSYSTEMS_DISPLAY_H
#define SUBSYSTEMS_DISPLAY_H

#include <Arduino.h>

// True once initDisplay() has confirmed the SSD1306 is present and
// initialized. Battery protection and motion still work fine without a
// display - this only gates whether updateDisplay()/showShutdownScreen()
// actually draw anything.
extern bool displayAvailable;

// Starts Wire2 and the SSD1306. Call once from setup(), after
// Wire2.begin().
void initDisplay();

// Redraws the normal status screen (battery voltage, uptime, run/idle
// timer). Does nothing once shutdownLatched is set, since
// showShutdownScreen() owns the screen permanently after that.
void updateDisplay();

// Non-blocking scheduler - calls updateDisplay() no more often than every
// DISPLAY_UPDATE_INTERVAL_MS. Call this every tick; systemTick() does.
void displayTick();

// Draws the permanent low-battery warning screen. Called once by
// battery.cpp's emergencyShutdown() - after this, updateDisplay() no
// longer touches the screen.
void showShutdownScreen(
  float voltage,
  float limitVoltage,
  unsigned long uptimeMs);

// Formats a millisecond duration as "MM:SS", or "H:MM:SS" once it
// crosses an hour (uptime can run long across a full competition day).
String formatDuration(unsigned long ms);

#endif
