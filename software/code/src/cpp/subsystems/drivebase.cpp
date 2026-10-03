#include "include/subsystems/drivebase.h"

#include "include/subsystems/imu.h"
#include "include/subsystems/localization.h"
#include "include/subsystems/robot_config.h"
#include "include/subsystems/robot_state.h"
#include "include/subsystems/strategy.h"

namespace {
float clampMagnitude(float x, float y, float maxMagnitude) {
  float magnitude = sqrtf(x * x + y * y);
  if (magnitude <= maxMagnitude || magnitude <= 0.0f) {
    return magnitude;
  }
  return maxMagnitude;
}

void slewVector(float targetX, float targetY, float &currentX, float &currentY,
                float maxChange) {
  float deltaX = targetX - currentX;
  float deltaY = targetY - currentY;
  float deltaMagnitude = sqrtf(deltaX * deltaX + deltaY * deltaY);
  if (deltaMagnitude > maxChange && deltaMagnitude > 0.0f) {
    float scale = maxChange / deltaMagnitude;
    deltaX *= scale;
    deltaY *= scale;
  }
  currentX += deltaX;
  currentY += deltaY;
}

float slewValue(float target, float current, float maxChange) {
  float delta = constrain(target - current, -maxChange, maxChange);
  return current + delta;
}

struct CoordinateController {
  bool initialized = false;
  float velocityX = 0.0f;
  float velocityY = 0.0f;
  float rotation = 0.0f;
  unsigned long lastUpdateMs = 0;
};

SIM_TLS CoordinateController coordinateController;
}

void moveTo(float targetXmm, float targetYmm, float targetHeadingDeg,
            float maxSpeed, float accelerationLimit,
            float maxRotationSpeed, float rotationAccelerationLimit) {
  if (!isIMUHeadingFresh()) {
    stopAllDriveMotors();
    return;
  }

  RobotPose pose;
  getRobotPose(pose);

  unsigned long now = millis();
  float dt = coordinateController.initialized
    ? (now - coordinateController.lastUpdateMs) / 1000.0f
    : 0.0f;
  dt = constrain(dt, 0.001f, 0.05f);

  // Rotation depends only on the IMU, so it keeps working without a lidar fix.
  if (!coordinateController.initialized) {
    resetHeadingPID();
  }
  desiredHeadingDeg = targetHeadingDeg;
  maxRotationSpeed = constrain(maxRotationSpeed, 0.0f, 1.0f);
  const float targetRotation = constrain(headingCorrection(),
                                         -maxRotationSpeed, maxRotationSpeed);
  rotationAccelerationLimit = fmaxf(0.0f, rotationAccelerationLimit);
  coordinateController.rotation = slewValue(
    targetRotation, coordinateController.rotation,
    rotationAccelerationLimit * dt);

  if (!pose.valid) {
    coordinateController.velocityX = 0.0f;
    coordinateController.velocityY = 0.0f;
    drive(0.0f, 0.0f, coordinateController.rotation);
    coordinateController.initialized = true;
    coordinateController.lastUpdateMs = now;
    return;
  }

  float errorXmm = targetXmm - pose.xMm;
  float errorYmm = targetYmm - pose.yMm;
  float distanceMm = sqrtf(errorXmm * errorXmm + errorYmm * errorYmm);
  // Only the physical normalized range applies; each call owns its own limits.
  maxSpeed = constrain(maxSpeed, 0.0f, 1.0f);
  accelerationLimit = fmaxf(0.0f, accelerationLimit);

  float targetSpeed = fminf(maxSpeed, POSITION_KP * distanceMm);
  if (accelerationLimit > 0.0f) {
    const float stoppingSpeed = sqrtf(
        2.0f * accelerationLimit * distanceMm / ROBOT_LINEAR_SPEED_MM_S);
    targetSpeed = fminf(targetSpeed, stoppingSpeed);
  } else {
    targetSpeed = 0.0f;
  }
  if (distanceMm > POSITION_TOLERANCE_MM && targetSpeed > 0.0f) {
    targetSpeed = fmaxf(targetSpeed, fminf(MOTOR_MIN_COMMAND, maxSpeed));
  }

  float targetVelocityX = 0.0f;
  float targetVelocityY = 0.0f;
  if (distanceMm > POSITION_TOLERANCE_MM) {
    targetVelocityX = targetSpeed * errorXmm / distanceMm;
    targetVelocityY = targetSpeed * errorYmm / distanceMm;
  } else {
    // Arrived: stop immediately instead of slewing down while coasting past.
    coordinateController.velocityX = 0.0f;
    coordinateController.velocityY = 0.0f;
  }
  slewVector(targetVelocityX, targetVelocityY, coordinateController.velocityX,
             coordinateController.velocityY, accelerationLimit * dt);

  float speed = clampMagnitude(coordinateController.velocityX,
                               coordinateController.velocityY, maxSpeed);
  float direction = atan2f(coordinateController.velocityY,
                           coordinateController.velocityX) * 180.0f / PI;
  if (speed <= 0.0001f) {
    direction = 0.0f;
  }
  drive(direction, speed, coordinateController.rotation);

  coordinateController.initialized = true;
  coordinateController.lastUpdateMs = now;
}

void drive(float direction_deg, float speed, float rotation) {
  speed = constrain(speed, 0.0f, 1.0f);
  rotation = constrain(rotation, -1.0f, 1.0f);

  float effectiveYaw = YAW_SIGN * currentYawDeg;

  float body_direction_rad =
    (direction_deg - effectiveYaw) * PI / 180.0f;

  float vx = speed * cos(body_direction_rad);
  float vy = speed * sin(body_direction_rad);

  float wheel_speeds[4];

  wheel_speeds[0] =
    -vx * sin(45 * PI / 180.0f) + vy * cos(45 * PI / 180.0f) + rotation;

  wheel_speeds[1] =
    -vx * sin(-45 * PI / 180.0f) + vy * cos(-45 * PI / 180.0f) + rotation;

  wheel_speeds[2] =
    -vx * sin(-135 * PI / 180.0f) + vy * cos(-135 * PI / 180.0f) + rotation;

  wheel_speeds[3] =
    -vx * sin(135 * PI / 180.0f) + vy * cos(135 * PI / 180.0f) + rotation;

  float max_speed = 0.0f;

  for (int i = 0; i < 4; i++) {
    if (fabs(wheel_speeds[i]) > max_speed) {
      max_speed = fabs(wheel_speeds[i]);
    }
  }

  float wheelScale = 1.0f;
  if (max_speed > 1.0f) {
    wheelScale = 1.0f / max_speed;
  } else if (speed >= MOTOR_MIN_COMMAND && max_speed > 0.02f &&
             max_speed < MOTOR_MIN_COMMAND) {
    // Lift weak commands above the motor deadband, keeping their direction.
    wheelScale = MOTOR_MIN_COMMAND / max_speed;
  } else if (max_speed <= 0.02f) {
    wheelScale = 0.0f;
  }
  for (int i = 0; i < 4; i++) {
    wheel_speeds[i] *= wheelScale;
  }

#ifdef DEBUG_MOVE
  static unsigned long lastDebugMs = 0;
  unsigned long nowMs = millis();
  if (nowMs - lastDebugMs >= 100) {
    lastDebugMs = nowMs;
    Serial.print("dir=");
    Serial.print(direction_deg);
    Serial.print(" yaw=");
    Serial.print(currentYawDeg);
    Serial.print(" bodyDir=");
    Serial.print(body_direction_rad * 180.0f / PI);
    Serial.print(" vx=");
    Serial.print(vx);
    Serial.print(" vy=");
    Serial.print(vy);
    Serial.print(" rot=");
    Serial.print(rotation);
    Serial.print(" w=[");
    Serial.print(wheel_speeds[0]);
    Serial.print(",");
    Serial.print(wheel_speeds[1]);
    Serial.print(",");
    Serial.print(wheel_speeds[2]);
    Serial.print(",");
    Serial.print(wheel_speeds[3]);
    Serial.println("]");
  }
#endif

  SetSpeed(1, wheel_speeds[0] * 255);
  SetSpeed(2, wheel_speeds[1] * 255);
  SetSpeed(3, wheel_speeds[3] * 255);
  SetSpeed(4, wheel_speeds[2] * 255);

  // Update MoveProfile so the simulator (or any reader) can observe the
  // commanded movement. RotationSpeed is the normalized rotation command
  // (signed, unitless) and is passed through directly.
  currentMoveProfile.active = true;
  currentMoveProfile.movementDirectionDeg = direction_deg;
  currentMoveProfile.speed = speed;
  currentMoveProfile.rotationSpeed = rotation;
  currentMoveProfile.lastUpdateMs = millis();
}

void stopAllDriveMotors() {
  SetSpeed(1, 0);
  SetSpeed(2, 0);
  SetSpeed(3, 0);
  SetSpeed(4, 0);
  coordinateController.initialized = false;
  coordinateController.velocityX = 0.0f;
  coordinateController.velocityY = 0.0f;
  coordinateController.rotation = 0.0f;
  currentMoveProfile.active = false;
}

void SetSpeed(int motor, int pwm) {
  pwm = (int)(pwm * motorMult[motor]);
  pwm = constrain(pwm, -255, 255);

  int pinA;
  int pinB;

  switch (motor) {
    case 1:
      pinA = M1a;
      pinB = M1b;
      break;

    case 2:
      pinA = M2a;
      pinB = M2b;
      break;

    case 3:
      pinA = M3a;
      pinB = M3b;
      break;

    case 4:
      pinA = M4a;
      pinB = M4b;
      break;

    default:
      return;
  }

  if (pwm > 0) {
    analogWrite(pinA, pwm);
    analogWrite(pinB, 0);
  } else if (pwm < 0) {
    analogWrite(pinA, 0);
    analogWrite(pinB, -pwm);
  } else {
    analogWrite(pinA, 0);
    analogWrite(pinB, 0);
  }
}
