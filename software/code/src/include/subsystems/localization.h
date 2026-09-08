#pragma once

#include <stdint.h>

struct RobotPose {
  bool valid;
  float xMm;
  float yMm;
  float headingDeg;
  float quality;
  unsigned long timestampMs;
};

struct FieldBall {
  bool valid;
  float xMm;
  float yMm;
  float distanceCm;
  float angleDeg;
  unsigned long timestampMs;
};

void initLocalization();
void updateLocalization();
void getRobotPose(RobotPose &out);
void getFieldBall(FieldBall &out);