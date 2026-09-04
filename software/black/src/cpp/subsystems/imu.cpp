#include "include/subsystems/imu.h"

#include <Wire.h>

#include "include/subsystems/robot_config.h"
#include "include/subsystems/robot_state.h"

Adafruit_BNO08x bno08x(BNO08X_RESET);
sh2_SensorValue_t sensorValue;

void initIMU() {

}

void setReports() {
  if (!bno08x.enableReport(SH2_GAME_ROTATION_VECTOR)) {
    Serial.println("Could not enable rotation vector");
  }
}

float quaternionToYawDegrees(float real, float i, float j, float k) {
  float yaw =
    atan2(
      2.0f * (real * k + i * j),
      1.0f - 2.0f * (j * j + k * k));

  return yaw * 180.0f / PI;
}

void updateIMU() {
  currentYawDeg = 0;
}

float angleError(float target, float current) {
  float error = target - current;

  while (error > 180.0f) {
    error -= 360.0f;
  }

  while (error < -180.0f) {
    error += 360.0f;
  }

  return error;
}

void resetHeadingPID() {
  headingIntegral = 0.0f;
  headingLastError = 0.0f;
  headingLastTimeMs = millis();
  headingPidInitialized = false;
}

float headingCorrection() {
  unsigned long now = millis();

  float error = angleError(desiredHeadingDeg, currentYawDeg);

  float dt = 0.0f;

  if (headingPidInitialized) {
    dt = (now - headingLastTimeMs) / 1000.0f;
  }

  if (dt > 0.0f) {
    headingIntegral += error * dt;
    headingIntegral =
      constrain(
        headingIntegral,
        -HEADING_INTEGRAL_MAX,
        HEADING_INTEGRAL_MAX);
  }

  float derivative = 0.0f;

  if (dt > 0.0f) {
    derivative = (error - headingLastError) / dt;
  }

  float correction =
    (error * HEADING_KP) +
    (headingIntegral * HEADING_KI) +
    (derivative * HEADING_KD);

  correction = constrain(correction, -0.40f, 0.40f);

  headingLastError = error;
  headingLastTimeMs = now;
  headingPidInitialized = true;

  return correction;
}
