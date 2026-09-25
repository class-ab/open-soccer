#pragma once

#include <Adafruit_BNO08x.h>

extern Adafruit_BNO08x bno08x;
extern sh2_SensorValue_t sensorValue;

// Returns false when the IMU is absent or cannot be reached on I2C.
bool initIMU();
void setReports();
void updateIMU();
float getIMUYawRateDegPerSec();
bool isIMUHeadingFresh();

float quaternionToYawDegrees(float real, float i, float j, float k);
float angleError(float target, float current);
float headingCorrection();
void resetHeadingPID();
