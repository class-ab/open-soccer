#include "../sim_hal/Arduino.h"
#include "sim_hal/sim_robot_io.h"
#include "subsystems/dribbler.h"
#include "subsystems/robot_config.h"
#include "subsystems/robot_state.h"

#include <atomic>

namespace {
struct KickState {
  unsigned long startedMs = 0;
  bool kicking = false;
  bool pulseSent = false;
  std::atomic<bool> requested{false};
};
KickState g_kickStates[2];
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
  KickState &k = g_kickStates[g_simRobotSlot];
  k.kicking = false;
  k.pulseSent = false;
}

void kick() {
  KickState &k = g_kickStates[g_simRobotSlot];
  if (k.kicking) return;
  k.startedMs = millis();
  k.kicking = true;
  k.pulseSent = false;
}

void updateKicker() {
  KickState &k = g_kickStates[g_simRobotSlot];
  if (!k.kicking) return;
  const unsigned long elapsed = millis() - k.startedMs;
  if (!k.pulseSent && elapsed >= 20) {
    k.requested.store(true);
    k.pulseSent = true;
  }
  if (elapsed >= 60) k.kicking = false;
}

void sim_request_kick(int slot) {
  g_kickStates[slot].requested.store(true);
}

bool sim_consume_kick_request(int slot) {
  return g_kickStates[slot].requested.exchange(false);
}
