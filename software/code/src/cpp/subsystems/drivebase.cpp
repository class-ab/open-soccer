#include "include/subsystems/drivebase.h"

#include "include/subsystems/imu.h"
#include "include/subsystems/localization.h"
#include "include/subsystems/robot_config.h"
#include "include/subsystems/robot_state.h"
#include "include/subsystems/robot_tick.h"

namespace {
float clampMagnitude(float x, float y, float maxMagnitude) {
  float magnitude = sqrtf(x * x + y * y);
  if (magnitude <= maxMagnitude || magnitude <= 0.0f) {
    return magnitude;
  }
  return maxMagnitude;
}

void limitVectorMagnitude(float &x, float &y, float maxMagnitude) {
  float magnitude = sqrtf(x * x + y * y);
  if (magnitude > maxMagnitude && magnitude > 0.0f) {
    float scale = maxMagnitude / magnitude;
    x *= scale;
    y *= scale;
  }
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
  float targetXmm = 0.0f;
  float targetYmm = 0.0f;
  float targetHeadingDeg = 0.0f;
  float integralXmm = 0.0f;
  float integralYmm = 0.0f;
  float lastErrorXmm = 0.0f;
  float lastErrorYmm = 0.0f;
  float filteredDerivativeX = 0.0f;
  float filteredDerivativeY = 0.0f;
  float velocityX = 0.0f;
  float velocityY = 0.0f;
  float rotation = 0.0f;
  unsigned long lastUpdateMs = 0;
};

CoordinateController coordinateController;
}

void moveTo(float targetXmm, float targetYmm, float targetHeadingDeg,
            float maxSpeed, float accelerationLimit,
            float maxRotationSpeed, float rotationAccelerationLimit) {
  RobotPose pose;
  getRobotPose(pose);

  unsigned long now = millis();
  if (!pose.valid) {
    coordinateController.initialized = false;
    coordinateController.velocityX = 0.0f;
    coordinateController.velocityY = 0.0f;
    coordinateController.rotation = 0.0f;
    currentMoveProfile.active = false;
    stopAllDriveMotors();
    return;
  }

  float dt = coordinateController.initialized
    ? (now - coordinateController.lastUpdateMs) / 1000.0f
    : 0.0f;
  dt = constrain(dt, 0.001f, 0.1f);

  bool newTarget = !coordinateController.initialized ||
                   targetXmm != coordinateController.targetXmm ||
                   targetYmm != coordinateController.targetYmm ||
                   targetHeadingDeg != coordinateController.targetHeadingDeg;
  if (newTarget) {
    coordinateController.integralXmm = 0.0f;
    coordinateController.integralYmm = 0.0f;
    coordinateController.lastErrorXmm = targetXmm - pose.xMm;
    coordinateController.lastErrorYmm = targetYmm - pose.yMm;
    coordinateController.filteredDerivativeX = 0.0f;
    coordinateController.filteredDerivativeY = 0.0f;
  }

  float errorXmm = targetXmm - pose.xMm;
  float errorYmm = targetYmm - pose.yMm;
  float derivativeX = (errorXmm - coordinateController.lastErrorXmm) / dt;
  float derivativeY = (errorYmm - coordinateController.lastErrorYmm) / dt;
  coordinateController.filteredDerivativeX +=
    POSITION_DERIVATIVE_FILTER * (derivativeX - coordinateController.filteredDerivativeX);
  coordinateController.filteredDerivativeY +=
    POSITION_DERIVATIVE_FILTER * (derivativeY - coordinateController.filteredDerivativeY);

  coordinateController.integralXmm = constrain(
    coordinateController.integralXmm + errorXmm * dt,
    -POSITION_INTEGRAL_MAX_MM, POSITION_INTEGRAL_MAX_MM);
  coordinateController.integralYmm = constrain(
    coordinateController.integralYmm + errorYmm * dt,
    -POSITION_INTEGRAL_MAX_MM, POSITION_INTEGRAL_MAX_MM);

  float targetVelocityX = POSITION_KP * errorXmm +
                          POSITION_KI * coordinateController.integralXmm +
                          POSITION_KD * coordinateController.filteredDerivativeX;
  float targetVelocityY = POSITION_KP * errorYmm +
                          POSITION_KI * coordinateController.integralYmm +
                          POSITION_KD * coordinateController.filteredDerivativeY;
  maxSpeed = constrain(maxSpeed, 0.0f, ROBOT_MAX_SPEED);
  limitVectorMagnitude(targetVelocityX, targetVelocityY, maxSpeed);

  float distanceMm = sqrtf(errorXmm * errorXmm + errorYmm * errorYmm);
  if (distanceMm <= POSITION_TOLERANCE_MM) {
    targetVelocityX = 0.0f;
    targetVelocityY = 0.0f;
  }
  accelerationLimit = fmaxf(0.0f, accelerationLimit);
  slewVector(targetVelocityX, targetVelocityY, coordinateController.velocityX,
             coordinateController.velocityY, accelerationLimit * dt);

  float headingError = angleError(targetHeadingDeg, pose.headingDeg);
  maxRotationSpeed = constrain(maxRotationSpeed, 0.0f, ROTATION_MAX_SPEED);
  float targetRotation = constrain(headingError * HEADING_KP,
                                   -maxRotationSpeed, maxRotationSpeed);
  if (fabsf(headingError) <= HEADING_TOLERANCE_DEG) {
    targetRotation = 0.0f;
  }
  targetRotation = constrain(targetRotation, -maxRotationSpeed, maxRotationSpeed);
  rotationAccelerationLimit = fmaxf(0.0f, rotationAccelerationLimit);
  coordinateController.rotation = slewValue(
    targetRotation, coordinateController.rotation,
    rotationAccelerationLimit * dt);

  float speed = clampMagnitude(coordinateController.velocityX,
                               coordinateController.velocityY, maxSpeed);
  float direction = atan2f(coordinateController.velocityY,
                           coordinateController.velocityX) * 180.0f / PI;
  if (speed <= 0.0001f) {
    direction = 0.0f;
  }
  drive(direction, speed, coordinateController.rotation);

  coordinateController.initialized = true;
  coordinateController.targetXmm = targetXmm;
  coordinateController.targetYmm = targetYmm;
  coordinateController.targetHeadingDeg = targetHeadingDeg;
  coordinateController.lastErrorXmm = errorXmm;
  coordinateController.lastErrorYmm = errorYmm;
  coordinateController.lastUpdateMs = now;
}

void drive(float direction_deg, float speed, float rotation) {
  speed = constrain(speed, 0.0f, ROBOT_MAX_SPEED);

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

  if (max_speed > 1.0f) {
    for (int i = 0; i < 4; i++) {
      wheel_speeds[i] /= max_speed;
    }
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
