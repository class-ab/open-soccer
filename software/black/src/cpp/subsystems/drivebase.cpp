#include "include/subsystems/drivebase.h"

#include "include/subsystems/imu.h"
#include "include/subsystems/robot_config.h"
#include "include/subsystems/robot_state.h"
#include "include/subsystems/robot_tick.h"

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

  SetSpeed(1, wheel_speeds[0] * 255 * ROBOT_MAX_SPEED);
  SetSpeed(2, wheel_speeds[1] * 255 * ROBOT_MAX_SPEED);
  SetSpeed(3, wheel_speeds[3] * 255 * ROBOT_MAX_SPEED);
  SetSpeed(4, wheel_speeds[2] * 255 * ROBOT_MAX_SPEED);

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
  if(pwm > 0 && pwm < 35) {
    pwm = 35;
  } else if(pwm < 0 && pwm > -35) {
    pwm = -35;
  }
  
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
