#include "subsystems/robot_state.h"

// Simple definitions of the globals the simulator will read/write.
// The real robot firmware provides its own definitions when linked; the
// simulator builds standalone and therefore defines them here.

BallPacket latestBallPacket = {false, 0.0f, 0.0f, 0};
GoalPacket latestYellowGoalPacket = {false, 0.0f, 0.0f, 0};
GoalPacket latestBlueGoalPacket = {false, 0.0f, 0.0f, 0};
unsigned long lastBallPacketMs = 0;
unsigned long lastYellowGoalPacketMs = 0;
unsigned long lastBlueGoalPacketMs = 0;

MoveProfile currentMoveProfile = {false, 0.0f, 0.0f, 0.0f, 0};

bool isYellowAlliance = true;
bool hasBall = false;
