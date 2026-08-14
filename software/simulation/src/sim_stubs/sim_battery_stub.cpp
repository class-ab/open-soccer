#include "../sim_hal/Arduino.h"
#include "../../code/src/include/subsystems/robot_state.h"

float readBatteryVoltage() {
  // return a safe value above any shutdown threshold
  return 20.0f;
}

void checkBattery() {
  lastBatteryVoltage = readBatteryVoltage();
  lastBatteryCheckMs = millis();
}

void emergencyShutdown() {
  // mark shutdown but don't block the sim
  shutdownLatched = true;
}
