#include "../sim_hal/Arduino.h"

// Provide the same symbols as dribbler.h without including the original header.
void initDribbler() {
  // no-op in sim
}

void setDribblerDirectionForward() {
  // no-op
}

void setDribblerDirectionReverse() {
  // no-op
}

void setDribblerThrottle(int throttleUs) {
  (void)throttleUs; // no-op
}

void stopDribbler() {
  // no-op
}
