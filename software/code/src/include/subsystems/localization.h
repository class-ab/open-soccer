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

struct OpponentRobot { // NEED TO SUBTRACT OWN ROBOT USING COMMUNICATION
  bool valid;
  float xMm;
  float yMm;
  float confidence;
  unsigned long timestampMs;
};

void initLocalization();
void updateLocalization();
void getRobotPose(RobotPose &out);
void getFieldBall(FieldBall &out);
void getOpponents(OpponentRobot *out, int maxOpponents, int &count);