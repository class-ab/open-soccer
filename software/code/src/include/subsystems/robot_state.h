#pragma once

#include <Arduino.h>

struct BallPacket {
  bool detected;
  float angleDeg;
  float radiusPx;
  uint8_t sizeByte;
};

struct MoveProfile {
  float accelTime;
  float cruiseTime;
  float decelTime;
  float peakSpeed;
};

struct RotationProfile {
  float accelTime;
  float cruiseTime;
  float decelTime;
  float peakOmegaMag;
  float effectiveAccelMag;
  float totalDelta;
  float rotationTime;
};

extern BallPacket latestBallPacket;
extern unsigned long lastBallPacketMs;
extern uint8_t ballPacketBuf[];
extern uint8_t ballPacketIdx;
extern bool ballSyncFound;

extern unsigned long bootMillis;
extern unsigned long lastRunStateChangeMs;
extern bool robotCurrentlyRunning;

extern unsigned long lastBatteryCheckMs;
extern float lastBatteryVoltage;
extern bool shutdownLatched;
extern bool dribblerShouldRun;

extern unsigned long lastDisplayUpdateMs;
extern bool displayAvailable;

extern float currentYawDeg;
extern float desiredHeadingDeg;
extern float headingIntegral;
extern float headingLastError;
extern unsigned long headingLastTimeMs;
extern bool headingPidInitialized;
