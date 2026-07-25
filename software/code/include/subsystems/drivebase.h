#pragma once

#include "subsystems/robot_state.h"

void SetSpeed(int motor, int pwm);
void stopAllMotors();

void drive(float direction_deg, float speed, float rotation);

float speedAtTime(
  const MoveProfile &profile,
  float t_sec,
  float totalTime_sec,
  float accelLimit);

float angleAtTime(const RotationProfile &profile, float t_sec);
