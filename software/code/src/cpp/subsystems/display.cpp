#include "include/subsystems/display.h"

#include <Wire.h>

#include "include/subsystems/robot_config.h"
#include "include/subsystems/robot_state.h"

Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire2, OLED_RESET_PIN);

void initDisplay() {
  displayAvailable = display.begin(SSD1306_SWITCHCAPVCC, OLED_I2C_ADDRESS);

  if (!displayAvailable) {
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

void updateDisplay() {
  if (!displayAvailable || shutdownLatched) {
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
  display.println(isYellowAlliance ? "Yellow" : "Blue");

  display.print(robotCurrentlyRunning ? "Run tmr: " : "Idle tmr:");
  display.println(formatDuration(runStateMs));

  display.display();
}

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
