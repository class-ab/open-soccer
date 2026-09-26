#pragma once

#ifdef ARDUINO
#include <Arduino.h>
#else
// When compiling for the simulator (not Arduino), use the simulator HAL
#include "sim_hal/Arduino.h"
#include <cstdint>
#include <cmath>
using uint8_t = std::uint8_t;
#endif

#include "robot_config.h"

struct BallPacket {
  bool detected;
  float angleDeg;   // degrees (relative to robot front/dribbler direction)
  float distanceCM; // centimeters
  uint8_t sizeByte;
};

// Simplified MoveProfile used by simulator and robot code
struct MoveProfile {
  bool active;                   // if true, simulator should use the profile
  float movementDirectionDeg;    // degrees (0 = front/dribbler direction)
  float speed;                   // speed (same units as drive() speed parameter)
  float rotationSpeed;          // normalized rotation speed (signed, unitless)
  unsigned long lastUpdateMs;    // millis() when last updated
};

extern SIM_TLS BallPacket latestBallPacket;
extern SIM_TLS unsigned long lastBallPacketMs;
extern SIM_TLS uint8_t ballPacketBuf[];
extern SIM_TLS uint8_t ballPacketIdx;
extern SIM_TLS bool ballSyncFound;

extern SIM_TLS MoveProfile currentMoveProfile;

extern SIM_TLS unsigned long bootMillis;
extern SIM_TLS unsigned long lastRunStateChangeMs;
extern SIM_TLS bool robotCurrentlyRunning;

extern SIM_TLS unsigned long lastBatteryCheckMs;
extern SIM_TLS float lastBatteryVoltage;
extern SIM_TLS bool shutdownLatched;
extern SIM_TLS bool dribblerShouldRun;

extern SIM_TLS unsigned long lastDisplayUpdateMs;
extern SIM_TLS bool displayAvailable;

extern SIM_TLS float currentYawDeg;
extern SIM_TLS float desiredHeadingDeg;
extern SIM_TLS float headingIntegral;
extern SIM_TLS float headingLastError;
extern SIM_TLS unsigned long headingLastTimeMs;
extern SIM_TLS bool headingPidInitialized;

void checkButtons();

extern SIM_TLS bool button1State;
extern SIM_TLS bool button2State;
extern SIM_TLS bool button3State;