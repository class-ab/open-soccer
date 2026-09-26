#pragma once

#include "subsystems/localization.h"

void sim_set_localization(const RobotPose &pose, const FieldBall &ball,
                          const OpponentRobot *opponents, int count);
void sim_set_imu_state(float headingDeg, float yawRateDegPerSec);
void sim_request_kick();
bool sim_consume_kick_request();