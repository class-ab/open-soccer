#pragma once

#include <Adafruit_BNO08x.h>

extern Adafruit_BNO08x bno08x;
extern sh2_SensorValue_t sensorValue;

void initIMU();
void setReports();
void updateIMU();

float quaternionToYawDegrees(float real, float i, float j, float k);
float angleError(float target, float current);
float headingCorrection();
void resetHeadingPID();
