#include "subsystems/drivebase.h"

#include "subsystems/imu.h"
#include "subsystems/robot_config.h"
#include "subsystems/robot_state.h"
#include "subsystems/robot_tick.h"

MoveProfile computeMoveProfile(
  float maxSpeed,
  float totalTime_sec,
  float accelLimit) {

  MoveProfile profile;

  maxSpeed = constrain(maxSpeed, 0.0f, 1.0f);

  if (accelLimit <= 0.0001f || totalTime_sec <= 0.0001f) {
    profile.accelTime = 0.0f;
    profile.decelTime = 0.0f;
    profile.cruiseTime = totalTime_sec;
    profile.peakSpeed = maxSpeed;
    return profile;
  }

  float accelTimeFull = maxSpeed / accelLimit;

  if ((accelTimeFull * 2.0f) <= totalTime_sec) {
    profile.accelTime = accelTimeFull;
    profile.decelTime = accelTimeFull;
    profile.cruiseTime = totalTime_sec - (accelTimeFull * 2.0f);
    profile.peakSpeed = maxSpeed;
  } else {
    profile.accelTime = totalTime_sec / 2.0f;
    profile.decelTime = totalTime_sec / 2.0f;
    profile.cruiseTime = 0.0f;
    profile.peakSpeed = accelLimit * profile.accelTime;
  }

  return profile;
}

float speedAtTime(
  const MoveProfile &profile,
  float t_sec,
  float totalTime_sec,
  float accelLimit) {

  if (t_sec >= totalTime_sec) {
    return 0.0f;
  }

  if (t_sec < profile.accelTime) {
    return accelLimit * t_sec;
  }

  float cruiseEnd = profile.accelTime + profile.cruiseTime;

  if (t_sec < cruiseEnd) {
    return profile.peakSpeed;
  }

  float tIntoDecel = t_sec - cruiseEnd;
  float speed = profile.peakSpeed - (accelLimit * tIntoDecel);

  return constrain(speed, 0.0f, profile.peakSpeed);
}

RotationProfile computeRotationProfile(
  float rotationDelta_deg,
  float accelLimit_degs2,
  float maxOmega_degs) {

  RotationProfile profile;
  profile.totalDelta = rotationDelta_deg;

  float absDelta = fabs(rotationDelta_deg);
  float maxOmega = fabs(maxOmega_degs);

  if (absDelta <= 0.0001f || accelLimit_degs2 <= 0.0001f ||
      maxOmega <= 0.0001f) {
    profile.accelTime = 0.0f;
    profile.cruiseTime = 0.0f;
    profile.decelTime = 0.0f;
    profile.peakOmegaMag = 0.0f;
    profile.effectiveAccelMag = 0.0f;
    profile.rotationTime = 0.0f;
    return profile;
  }

  float a = accelLimit_degs2;
  float triangleDist = (maxOmega * maxOmega) / a;

  if (absDelta <= triangleDist) {
    float t_a = sqrt(absDelta / a);

    profile.accelTime = t_a;
    profile.decelTime = t_a;
    profile.cruiseTime = 0.0f;
    profile.peakOmegaMag = a * t_a;
    profile.effectiveAccelMag = a;
    profile.rotationTime = 2.0f * t_a;
  } else {
    float t_a = maxOmega / a;
    float cruiseDist = absDelta - triangleDist;
    float cruiseTime = cruiseDist / maxOmega;

    profile.accelTime = t_a;
    profile.decelTime = t_a;
    profile.cruiseTime = cruiseTime;
    profile.peakOmegaMag = maxOmega;
    profile.effectiveAccelMag = a;
    profile.rotationTime = (2.0f * t_a) + cruiseTime;
  }

  return profile;
}

float angleAtTime(const RotationProfile &profile, float t_sec) {
  float absDelta = fabs(profile.totalDelta);

  if (profile.rotationTime <= 0.0001f || absDelta <= 0.0001f) {
    return profile.totalDelta;
  }

  float sign = (profile.totalDelta >= 0.0f) ? 1.0f : -1.0f;

  t_sec = constrain(t_sec, 0.0f, profile.rotationTime);

  float mag;
  float a = profile.effectiveAccelMag;

  if (t_sec < profile.accelTime) {
    mag = 0.5f * a * t_sec * t_sec;
  } else {
    float cruiseEnd = profile.accelTime + profile.cruiseTime;
    float magAtAccelEnd = 0.5f * a * profile.accelTime * profile.accelTime;

    if (t_sec < cruiseEnd) {
      mag = magAtAccelEnd + (profile.peakOmegaMag * (t_sec - profile.accelTime));
    } else {
      float magAtCruiseEnd =
        magAtAccelEnd + (profile.peakOmegaMag * profile.cruiseTime);
      float tIntoDecel = t_sec - cruiseEnd;
      mag =
        magAtCruiseEnd +
        (profile.peakOmegaMag * tIntoDecel) -
        (0.5f * a * tIntoDecel * tIntoDecel);
    }
  }

  mag = constrain(mag, 0.0f, absDelta);

  return sign * mag;
}

void move(
  float direction_deg,
  unsigned long duration_ms,
  float targetRotation_deg,
  float maxSpeed) {

  if (duration_ms == 0) {
    stopAllMotors();
    return;
  }

  updateIMU();

  float startYaw = currentYawDeg;
  float rotationDelta = angleError(targetRotation_deg, startYaw);
  float totalTime_sec = duration_ms / 1000.0f;

  MoveProfile profile =
    computeMoveProfile(maxSpeed, totalTime_sec, ACCEL_LIMIT);

  RotationProfile rotationProfile =
    computeRotationProfile(rotationDelta, ROTATION_ACCEL_LIMIT, ROTATION_MAX_SPEED);

  resetHeadingPID();

  unsigned long moveStart = millis();

  while (true) {
    unsigned long elapsed_ms = millis() - moveStart;

    if (elapsed_ms >= duration_ms) {
      break;
    }

    systemTick();
    updateIMU();

    float t_sec = elapsed_ms / 1000.0f;
    float speed = speedAtTime(profile, t_sec, totalTime_sec, ACCEL_LIMIT);

    desiredHeadingDeg =
      startYaw + angleAtTime(rotationProfile, t_sec);

    float rotation = headingCorrection();

    drive(direction_deg, speed, rotation);

    delay(5);
  }

  stopAllMotors();
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
}

void stopAllMotors() {
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
