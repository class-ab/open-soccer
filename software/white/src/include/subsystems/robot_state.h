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

struct BallPacket {
  bool detected;
  float angleDeg;   // degrees (relative to robot front/dribbler direction)
  float distanceCM; // centimeters
  uint8_t sizeByte;
};

// Goal data shares the exact same 8-byte wire format as ball data
// (sync byte + detected + angle + distance + size + checksum), so the
// decode/storage mirrors BallPacket. Yellow goal and blue goal are kept
// separate so callers can tell them apart.
struct GoalPacket {
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

extern BallPacket latestBallPacket;
extern GoalPacket latestYellowGoalPacket;
extern GoalPacket latestBlueGoalPacket;
extern unsigned long lastBallPacketMs;
extern unsigned long lastBallSeenMs;
extern unsigned long lastYellowGoalPacketMs;
extern unsigned long lastBlueGoalPacketMs;

// Traffic for all three tracked colors shares a single UART (Serial7 /
// BALL_UART) so they share one framing state machine. Each packet is
// identified by its sync byte and dispatched to the matching store.
extern uint8_t visionPacketBuf[];
extern uint8_t visionPacketIdx;
extern bool visionSyncFound;

extern MoveProfile currentMoveProfile;

extern unsigned long bootMillis;
extern unsigned long lastRunStateChangeMs;
extern bool robotCurrentlyRunning;
extern bool isYellowAlliance;
extern bool hasBall;
extern bool isAttacking;

extern unsigned long lastBatteryCheckMs;
extern float lastBatteryVoltage;
extern unsigned long batteryLowSinceMs;
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
