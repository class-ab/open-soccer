#include "include/subsystems/robot_state.h"
#include "include/subsystems/robot_config.h"

BallPacket latestBallPacket = {false, 0.0f, 0.0f, 0};
GoalPacket latestYellowGoalPacket = {false, 0.0f, 0.0f, 0};
GoalPacket latestBlueGoalPacket = {false, 0.0f, 0.0f, 0};
unsigned long lastBallPacketMs = 0;
unsigned long lastYellowGoalPacketMs = 0;
unsigned long lastBlueGoalPacketMs = 0;
uint8_t visionPacketBuf[BALL_PACKET_LEN];
uint8_t visionPacketIdx = 0;
bool visionSyncFound = false;

MoveProfile currentMoveProfile = {false, 0.0f, 0.0f, 0.0f, 0};

unsigned long bootMillis = 0;
unsigned long lastRunStateChangeMs = 0;
bool robotCurrentlyRunning = false;
bool isYellowAlliance = true;
bool hasBall = false;

unsigned long lastBatteryCheckMs = 0;
float lastBatteryVoltage = 0.0f;
bool shutdownLatched = false;
bool dribblerShouldRun = false;

unsigned long lastDisplayUpdateMs = 0;
bool displayAvailable = false;

float currentYawDeg = 0.0f;
float desiredHeadingDeg = 0.0f;
float headingIntegral = 0.0f;
float headingLastError = 0.0f;
unsigned long headingLastTimeMs = 0;
bool headingPidInitialized = false;
