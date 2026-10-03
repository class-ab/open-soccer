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
constexpr unsigned long IMU_TIMEOUT_MS = 100;

float yawZeroDeg = 0.0f;
uint32_t lastYawSampleUs = 0;
bool yawSampleCaptured = false;
bool rebaseNextYawSample = false;

// Heading PID state; derivative only advances when a new IMU sample arrives.
uint32_t headingLastSampleUs = 0;
float headingDerivative = 0.0f;

float wrapDegrees(float angle) {
  angle = fmodf(angle + 180.0f, 360.0f);
  if (angle < 0.0f) angle += 360.0f;
  return angle - 180.0f;
}

// Yaw is relative to the heading at boot; a BNO reset keeps the current yaw continuous.
void updateYawFromQuaternion(float rawYawDeg, uint32_t nowUs) {
  if (!yawSampleCaptured) {
    yawZeroDeg = rawYawDeg;
    yawSampleCaptured = true;
  } else if (rebaseNextYawSample) {
    yawZeroDeg = wrapDegrees(rawYawDeg - currentYawDeg);
    rebaseNextYawSample = false;
  }
  currentYawDeg = wrapDegrees(rawYawDeg - yawZeroDeg);
  lastYawSampleUs = nowUs;
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
  const uint32_t nowUs = micros();
  if (bno08x.wasReset()) {
    setReports();
    rebaseNextYawSample = true;
  }

  while (bno08x.getSensorEvent(&sensorValue)) {
    if (sensorValue.sensorId != SH2_GAME_ROTATION_VECTOR) continue;
    const float rawYawDeg = quaternionToYawDegrees(
        sensorValue.un.gameRotationVector.real,
        sensorValue.un.gameRotationVector.i,
        sensorValue.un.gameRotationVector.j,
        sensorValue.un.gameRotationVector.k);
    updateYawFromQuaternion(rawYawDeg, nowUs);
  }
}

float getIMUHeadingDeg() {
  return currentYawDeg;
}

bool isIMUHeadingFresh() {
  return yawSampleCaptured && micros() - lastYawSampleUs <= IMU_TIMEOUT_MS * 1000U;
}

float angleError(float target, float current) {
  return wrapDegrees(target - current);
}

void resetHeadingPID() {
  headingIntegral = 0.0f;
  headingLastError = 0.0f;
  headingLastTimeMs = millis();
  headingPidInitialized = false;
  headingDerivative = 0.0f;
}

float headingCorrection() {
  const float error = angleError(desiredHeadingDeg, YAW_SIGN * currentYawDeg);

  if (!headingPidInitialized) {
    headingLastError = error;
    headingLastSampleUs = lastYawSampleUs;
    headingDerivative = 0.0f;
    headingPidInitialized = true;
  } else if (lastYawSampleUs != headingLastSampleUs) {
    const float dt = (lastYawSampleUs - headingLastSampleUs) / 1000000.0f;
    if (dt > 0.0f && dt <= 0.1f) {
      headingIntegral = constrain(headingIntegral + error * dt,
                                  -HEADING_INTEGRAL_MAX, HEADING_INTEGRAL_MAX);
      headingDerivative = angleError(error, headingLastError) / dt;
    } else {
      headingDerivative = 0.0f;
    }
    headingLastError = error;
    headingLastSampleUs = lastYawSampleUs;
  }

  const float correction = error * HEADING_KP +
                           headingIntegral * HEADING_KI +
                           headingDerivative * HEADING_KD;
  return constrain(correction, -0.40f, 0.40f);
}
