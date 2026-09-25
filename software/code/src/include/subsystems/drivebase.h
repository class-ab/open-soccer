#pragma once

#include "include/subsystems/robot_state.h"

void SetSpeed(int motor, int pwm);
void stopAllDriveMotors();

void drive(float direction_deg, float speed, float rotation);

// Move in field coordinates (millimeters) while converging on targetHeadingDeg.
// Limits are per-command and use normalized velocity units for translation and
// rotation, with acceleration expressed in those units per second.
void moveTo(float targetXmm, float targetYmm, float targetHeadingDeg,
			float maxSpeed, float accelerationLimit,
			float maxRotationSpeed, float rotationAccelerationLimit);
