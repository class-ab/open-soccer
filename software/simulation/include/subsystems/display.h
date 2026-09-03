#pragma once

#include <string>
#include "include/subsystems/robot_state.h"

// Minimal stub for display functions used by robot code when running in simulator
void initDisplay();
void updateDisplay();
std::string formatDuration(unsigned long ms);

// Provide a minimal Display-like API used in other files (no-op)
class Adafruit_SSD1306 {
public:
  Adafruit_SSD1306(int w, int h, void* wire, int reset) {}
  bool begin(int, unsigned long) { return false; }
  void clearDisplay() {}
  void setRotation(int) {}
  void setTextColor(int) {}
  void setTextSize(int) {}
  void setCursor(int, int) {}
  void println(const std::string&) {}
  void print(const std::string&) {}
  void display() {}
};

#define SSD1306_SWITCHCAPVCC 0
#define SSD1306_WHITE 1
