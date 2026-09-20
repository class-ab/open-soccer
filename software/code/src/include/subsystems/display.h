#pragma once

#include <Adafruit_SSD1306.h>

extern Adafruit_SSD1306 display;

void initDisplay();
// Shows progress or a fault while setup() is running.  This deliberately does
// not depend on the normal state data, which is not available during boot.
void showBootStatus(const char *line1, const char *line2 = nullptr);
void updateDisplay();
String formatDuration(unsigned long ms);
