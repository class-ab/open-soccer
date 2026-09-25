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

struct OpponentRobot {
  bool valid;
  float xMm;
  float yMm;
  float confidence;
  unsigned long timestampMs;
};

void initLocalization();
void updateLocalization();
// Supply the latest field-frame velocity command for continuous pose prediction.
void setLocalizationMotionCommand(float directionDeg, float speed);
void getRobotPose(RobotPose &out);
// Returns the latest predicted and LiDAR-corrected pose.
void getPredictedRobotPose(RobotPose &out);
void getFieldBall(FieldBall &out);
void getOpponents(OpponentRobot *out, int maxOpponents, int &count);
