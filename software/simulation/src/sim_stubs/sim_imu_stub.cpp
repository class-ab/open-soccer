#include "../sim_hal/Arduino.h"
#include "subsystems/imu.h"
#include "subsystems/robot_state.h"

void initIMU() {
  // start with yaw = 0
  currentYawDeg = 0.0f;
}

void setReports() {
  // no-op
}

void updateIMU() {
  // no-op; leave currentYawDeg as-is
}

float headingCorrection() {
  // simple stub: use existing logic from imu but without sensors
  unsigned long now = millis();
  float error = desiredHeadingDeg - currentYawDeg;
  while (error > 180.0f) error -= 360.0f;
  while (error < -180.0f) error += 360.0f;
  // very simple proportional correction
  float correction = (error * 0.005f);
  if (correction > 0.40f) correction = 0.40f;
  if (correction < -0.40f) correction = -0.40f;
  return correction;
}
