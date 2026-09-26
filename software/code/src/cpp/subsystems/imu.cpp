#include "include/subsystems/imu.h"

#include <math.h>
#include <Wire.h>

#include "include/subsystems/robot_config.h"
#include "include/subsystems/robot_state.h"

Adafruit_BNO08x bno08x(BNO08X_RESET);
sh2_SensorValue_t sensorValue;

namespace {
constexpr float LOCAL_RAD_TO_DEG = 180.0f / PI;
constexpr uint32_t IMU_REPORT_INTERVAL_US = 10000;  // 100 Hz
constexpr float YAW_RATE_FILTER = 0.35f;
constexpr float MAX_YAW_RATE_DEG_S = 900.0f;
constexpr unsigned long MAX_YAW_PREDICTION_MS = 20;
constexpr unsigned long IMU_TIMEOUT_MS = 100;

float previousRawYawDeg = 0.0f;
float unwrappedYawDeg = 0.0f;
float yawRateDegPerSec = 0.0f;
unsigned long lastYawSampleMs = 0;
bool yawSampleCaptured = false;
bool rebaseNextYawSample = false;

float wrapDegrees(float angle) {
  angle = fmodf(angle + 180.0f, 360.0f);
  if (angle < 0.0f) angle += 360.0f;
  return angle - 180.0f;
}

void updateYawFromQuaternion(float rawYawDeg, unsigned long now) {
  if (!yawSampleCaptured) {
    previousRawYawDeg = rawYawDeg;
    unwrappedYawDeg = 0.0f;
    lastYawSampleMs = now;
    yawRateDegPerSec = 0.0f;
    yawSampleCaptured = true;
  } else if (rebaseNextYawSample) {
    previousRawYawDeg = rawYawDeg;
    lastYawSampleMs = now;
    yawRateDegPerSec = 0.0f;
    rebaseNextYawSample = false;
  } else {
    const unsigned long elapsedMs = now - lastYawSampleMs;
    if (elapsedMs > 0) {
      const float delta = wrapDegrees(rawYawDeg - previousRawYawDeg);
      const float rate = delta * 1000.0f / elapsedMs;
      if (fabsf(rate) <= MAX_YAW_RATE_DEG_S) {
        unwrappedYawDeg += delta;
        yawRateDegPerSec +=
            YAW_RATE_FILTER * (rate - yawRateDegPerSec);
      } else {
        // A discontinuity is more likely an IMU restart/glitch than real motion.
        yawRateDegPerSec = 0.0f;
      }
      previousRawYawDeg = rawYawDeg;
      lastYawSampleMs = now;
    }
  }

  const unsigned long predictionMs =
      (now - lastYawSampleMs < MAX_YAW_PREDICTION_MS)
          ? now - lastYawSampleMs
          : MAX_YAW_PREDICTION_MS;
  currentYawDeg = wrapDegrees(
      unwrappedYawDeg + yawRateDegPerSec * predictionMs / 1000.0f);
}
}  // namespace

bool initIMU() {
  if (!bno08x.begin_I2C()) {
    Serial.println("BNO08x not found!");
    return false;
  }

  Serial.println("BNO08x Found");
  setReports();
  delay(500);

  const unsigned long sampleWaitStartMs = millis();
  while (!yawSampleCaptured && millis() - sampleWaitStartMs < 1000) {
    updateIMU();
    delay(1);
  }
  if (!yawSampleCaptured) {
    Serial.println("IMU yaw zero pending first sample");
  } else {
    Serial.println("IMU yaw zeroed at startup");
  }
  return true;
}

void setReports() {
  if (!bno08x.enableReport(SH2_GAME_ROTATION_VECTOR,
                           IMU_REPORT_INTERVAL_US)) {
    Serial.println("Could not enable rotation vector");
  }
}

float quaternionToYawDegrees(float real, float i, float j, float k) {
  const float yaw = atan2f(2.0f * (real * k + i * j),
                           1.0f - 2.0f * (j * j + k * k));
  return yaw * LOCAL_RAD_TO_DEG;
}

void updateIMU() {
  const unsigned long now = millis();
  if (bno08x.wasReset()) {
    setReports();
    rebaseNextYawSample = true;
  }

  if (bno08x.getSensorEvent(&sensorValue) &&
      sensorValue.sensorId == SH2_GAME_ROTATION_VECTOR) {
    const float rawYawDeg = quaternionToYawDegrees(
        sensorValue.un.gameRotationVector.real,
        sensorValue.un.gameRotationVector.i,
        sensorValue.un.gameRotationVector.j,
        sensorValue.un.gameRotationVector.k);
    updateYawFromQuaternion(rawYawDeg, now);
  } else if (yawSampleCaptured) {
    const unsigned long predictionMs =
        (now - lastYawSampleMs < MAX_YAW_PREDICTION_MS)
            ? now - lastYawSampleMs
            : MAX_YAW_PREDICTION_MS;
    currentYawDeg = wrapDegrees(
        unwrappedYawDeg + yawRateDegPerSec * predictionMs / 1000.0f);
  }
}

float getIMUHeadingDeg() {
  return currentYawDeg;
}

float getIMUYawRateDegPerSec() {
  return yawRateDegPerSec;
}

bool isIMUHeadingFresh() {
  return yawSampleCaptured && millis() - lastYawSampleMs <= IMU_TIMEOUT_MS;
}

float angleError(float target, float current) {
  return wrapDegrees(target - current);
}

void resetHeadingPID() {
  headingIntegral = 0.0f;
  headingLastError = 0.0f;
  headingLastTimeMs = millis();
  headingPidInitialized = false;
}

float headingCorrection() {
  const unsigned long now = millis();
  const float error = angleError(desiredHeadingDeg, YAW_SIGN * currentYawDeg);
  const float correction =
      error * HEADING_KP -
      YAW_SIGN * getIMUYawRateDegPerSec() * HEADING_KD;

  headingLastError = error;
  headingLastTimeMs = now;
  headingPidInitialized = true;
  headingIntegral = 0.0f;
  return constrain(correction, -0.40f, 0.40f);
}
