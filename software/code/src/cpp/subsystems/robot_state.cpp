#include "include/subsystems/robot_state.h"
#include "include/subsystems/robot_config.h"

BallPacket latestBallPacket = {false, 0.0f, 0.0f, 0};
unsigned long lastBallPacketMs = 0;
uint8_t ballPacketBuf[BALL_PACKET_LEN];
uint8_t ballPacketIdx = 0;
bool ballSyncFound = false;

unsigned long bootMillis = 0;
unsigned long lastRunStateChangeMs = 0;
bool robotCurrentlyRunning = false;

unsigned long lastBatteryCheckMs = 0;
float lastBatteryVoltage = 0.0f;
bool shutdownLatched = false;

unsigned long lastDisplayUpdateMs = 0;
bool displayAvailable = false;

float currentYawDeg = 0.0f;
float desiredHeadingDeg = 0.0f;
float headingIntegral = 0.0f;
float headingLastError = 0.0f;
unsigned long headingLastTimeMs = 0;
bool headingPidInitialized = false;
