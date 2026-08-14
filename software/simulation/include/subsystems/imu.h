#pragma once

// Minimal IMU header stub for simulator builds. Avoids including Adafruit_BNO08x.

extern void initIMU();
extern void setReports();
extern void updateIMU();

extern float headingCorrection();
