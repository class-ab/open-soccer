#pragma once
// Minimal stub to satisfy includes in display.h for simulator builds.
class Adafruit_SSD1306 {
public:
  Adafruit_SSD1306(int w, int h, void* wire, int reset) {}
  bool begin(int, unsigned long) { return false; }
  void clearDisplay() {}
  void setRotation(int) {}
  void setTextColor(int) {}
  void setTextSize(int) {}
  void setCursor(int, int) {}
  void println(const char*) {}
  void print(const char*) {}
  void display() {}
};
