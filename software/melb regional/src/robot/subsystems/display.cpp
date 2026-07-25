#include "display.h"

#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>

#include "robot_state.h"
#include "battery.h"

// ============================================================
// OLED Display (SSD1306 over I2C)
// ============================================================

// Runs on Wire2, Teensy 4.1's native second-alternate I2C bus, which
// defaults to pins 24/25 - i.e. A10 (SDA2) / A11 (SCL2). This is a
// separate bus from the BNO08x (which stays on the default Wire /
// pins 18-19), so the two devices never contend for the bus.
//
// Change SCREEN_WIDTH/HEIGHT/OLED_I2C_ADDRESS if your module differs
// (0x3C is the common address for 128x64 and 128x32 SSD1306 boards;
// some 128x32 boards use 0x3D instead).
static const int SCREEN_WIDTH = 128;
static const int SCREEN_HEIGHT = 64;
static const int OLED_RESET_PIN = -1; // most small SSD1306 boards have no reset pin
static const uint8_t OLED_I2C_ADDRESS = 0x3C;

static Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire2, OLED_RESET_PIN);

bool displayAvailable = false; // set true here if begin() succeeds

static const unsigned long DISPLAY_UPDATE_INTERVAL_MS = 250;
static unsigned long lastDisplayUpdateMs = 0;

void initDisplay() {
  displayAvailable = display.begin(SSD1306_SWITCHCAPVCC, OLED_I2C_ADDRESS);

  if (!displayAvailable) {
    // Not treated as fatal - battery protection and motion still work
    // fine without a display, this just means no on-robot readout.
    Serial.println("SSD1306 not found!");
    return;
  }

  display.clearDisplay();
  display.setRotation(2);
  display.setTextColor(SSD1306_WHITE);
  display.setTextSize(1);
  display.setCursor(0, 0);
  display.println("Booting...");
  display.display();
}

// Redraws the normal status screen: battery voltage, uptime since boot,
// and a run/idle timer (counts up while a move sequence is active,
// counts up from zero again once it stops - i.e. "how long has it been
// running" while running, "how long since it stopped" while idle).
void updateDisplay() {
  if (!displayAvailable || shutdownLatched) {
    // showShutdownScreen() owns the screen permanently once tripped.
    return;
  }

  unsigned long now = millis();
  unsigned long uptimeMs = now - bootMillis;
  unsigned long runStateMs = now - lastRunStateChangeMs;

  display.clearDisplay();
  display.setTextSize(1);
  display.setCursor(0, 0);

  display.print("Batt:   ");
  display.print(lastBatteryVoltage, 2);
  display.println("V");

  display.print("Uptime: ");
  display.println(formatDuration(uptimeMs));

  display.print("Status: ");
  display.println(robotCurrentlyRunning ? "RUNNING" : "STOPPED");

  display.print(robotCurrentlyRunning ? "Run tmr: " : "Idle tmr:");
  display.println(formatDuration(runStateMs));

  display.display();
}

void displayTick() {
  unsigned long now = millis();

  if (now - lastDisplayUpdateMs >= DISPLAY_UPDATE_INTERVAL_MS) {
    lastDisplayUpdateMs = now;
    updateDisplay();
  }
}

void showShutdownScreen(
  float voltage,
  float limitVoltage,
  unsigned long uptimeMs) {
  if (!displayAvailable) {
    return;
  }

  display.clearDisplay();
  display.setTextSize(1);
  display.setCursor(0, 0);
  display.println("!! LOW BATTERY !!");
  display.println("   SHUTDOWN");
  display.println("");
  display.print("Batt:  ");
  display.print(voltage, 2);
  display.println("V");
  display.print("Limit: ");
  display.print(limitVoltage, 1);
  display.println("V");
  display.print("Uptime: ");
  display.println(formatDuration(uptimeMs));
  display.println("");
  display.println("Power cycle to reset");
  display.display();
}

// Formats a millisecond duration as "MM:SS", or "H:MM:SS" once it
// crosses an hour (uptime can run long across a full competition day).
String formatDuration(unsigned long ms) {
  unsigned long totalSeconds = ms / 1000UL;
  unsigned long hours = totalSeconds / 3600UL;
  unsigned long minutes = (totalSeconds % 3600UL) / 60UL;
  unsigned long seconds = totalSeconds % 60UL;

  char buf[16];

  if (hours > 0) {
    snprintf(buf, sizeof(buf), "%lu:%02lu:%02lu", hours, minutes, seconds);
  } else {
    snprintf(buf, sizeof(buf), "%02lu:%02lu", minutes, seconds);
  }

  return String(buf);
}
