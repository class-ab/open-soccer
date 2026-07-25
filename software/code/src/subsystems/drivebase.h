#pragma once

#include "subsystems/robot_state.h"

void SetSpeed(int motor, int pwm);
void stopAllMotors();

void drive(float direction_deg, float speed, float rotation);

void move(
  float direction_deg,
  unsigned long duration_ms,
  float targetRotation_deg,
  float maxSpeed);

MoveProfile computeMoveProfile(
  float maxSpeed,
  float totalTime_sec,
  float accelLimit);

float speedAtTime(
  const MoveProfile &profile,
  float t_sec,
  float totalTime_sec,
  float accelLimit);

RotationProfile computeRotationProfile(
  float rotationDelta_deg,
  float accelLimit_degs2,
  float maxOmega_degs);

float angleAtTime(const RotationProfile &profile, float t_sec);
