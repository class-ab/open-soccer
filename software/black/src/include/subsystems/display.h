#pragma once

#include <Adafruit_SSD1306.h>

extern Adafruit_SSD1306 display;

void initDisplay();
void updateDisplay();
String formatDuration(unsigned long ms);
