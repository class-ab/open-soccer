#pragma once

#include "include/subsystems/robot_state.h"

void SetSpeed(int motor, int pwm);
void stopAllDriveMotors();

void drive(float direction_deg, float speed, float rotation);
