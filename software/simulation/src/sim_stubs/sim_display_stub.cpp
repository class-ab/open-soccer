#include "../sim_hal/Arduino.h"
#include "../../include/subsystems/display.h"
#include "../../code/src/include/subsystems/robot_state.h"

void initDisplay() {
  // mark display as unavailable in sim
  displayAvailable = false;
}

void updateDisplay() {
  // no-op in sim
}

String formatDuration(unsigned long ms) {
  unsigned long totalSeconds = ms / 1000UL;
  unsigned long hours = totalSeconds / 3600UL;
  unsigned long minutes = (totalSeconds % 3600UL) / 60UL;
  unsigned long seconds = totalSeconds % 60UL;
  char buf[32];
  if (hours > 0) snprintf(buf, sizeof(buf), "%lu:%02lu:%02lu", hours, minutes, seconds);
  else snprintf(buf, sizeof(buf), "%02lu:%02lu", minutes, seconds);
  return String(buf);
}
