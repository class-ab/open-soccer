#include "../sim_hal/Arduino.h"
#include "sim_hal/sim_robot_io.h"
#include "subsystems/dribbler.h"
#include "subsystems/robot_config.h"
#include "subsystems/robot_state.h"

namespace {
unsigned long kickStartedMs = 0;
bool kicking = false;
bool kickPulseSent = false;
bool kickRequested = false;
}

// Simulator-side implementation of dribbler and kicker outputs.
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
  dribblerShouldRun = throttleUs > DRIBBLER_PULSE_MIN;
}

void stopDribbler() {
  dribblerShouldRun = false;
}

void initKicker() {
  kicking = false;
  kickPulseSent = false;
}

void kick() {
  if (kicking) return;
  kickStartedMs = millis();
  kicking = true;
  kickPulseSent = false;
}

void updateKicker() {
  if (!kicking) return;
  const unsigned long elapsed = millis() - kickStartedMs;
  if (!kickPulseSent && elapsed >= 20) {
    kickRequested = true;
    kickPulseSent = true;
  }
  if (elapsed >= 60) kicking = false;
}

void sim_request_kick() {
  kickRequested = true;
}

bool sim_consume_kick_request() {
  const bool requested = kickRequested;
  kickRequested = false;
  return requested;
}
