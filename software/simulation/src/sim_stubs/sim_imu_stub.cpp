#include "sim_hal/Arduino.h"
#include "sim_hal/sim_robot_io.h"
#include "subsystems/imu.h"
#include "subsystems/robot_config.h"
#include "subsystems/robot_state.h"

#include <cmath>
#include <mutex>

namespace {
std::mutex imuMutex;
float simulatedHeadingDeg[2] = {0.0f, 0.0f};
float simulatedYawRateDegPerSec[2] = {0.0f, 0.0f};
}

bool initIMU() {
  std::lock_guard<std::mutex> lock(imuMutex);
  currentYawDeg = simulatedHeadingDeg[g_simRobotSlot];
  return true;
}

void setReports() {
  // no-op
}

void updateIMU() {
  std::lock_guard<std::mutex> lock(imuMutex);
  currentYawDeg = simulatedHeadingDeg[g_simRobotSlot];
}

float getIMUHeadingDeg() {
  return currentYawDeg;
}

float getIMUYawRateDegPerSec() {
  std::lock_guard<std::mutex> lock(imuMutex);
  return simulatedYawRateDegPerSec[g_simRobotSlot];
}

bool isIMUHeadingFresh() {
  return true;
}

float quaternionToYawDegrees(float real, float i, float j, float k) {
  const float yaw = std::atan2(2.0f * (real * k + i * j),
                               1.0f - 2.0f * (j * j + k * k));
  return yaw * 180.0f / 3.14159265358979323846f;
}

float angleError(float target, float current) {
  float error = std::fmod(target - current + 540.0f, 360.0f) - 180.0f;
  return error;
}

void resetHeadingPID() {
  headingIntegral = 0.0f;
  headingLastError = 0.0f;
  headingPidInitialized = false;
}

float headingCorrection() {
  const float error = angleError(desiredHeadingDeg, currentYawDeg);
  return constrain(error * HEADING_KP -
                       YAW_SIGN * getIMUYawRateDegPerSec() * HEADING_KD,
                   -0.40f, 0.40f);
}

void sim_set_imu_state(int slot, float headingDeg, float yawRateDegPerSec) {
  std::lock_guard<std::mutex> lock(imuMutex);
  simulatedHeadingDeg[slot] = headingDeg;
  simulatedYawRateDegPerSec[slot] = yawRateDegPerSec;
}
