#pragma once

#include "include/subsystems/robot_state.h"

void initBallTracking();
bool isBallSyncByte(uint8_t b);
bool isYellowGoalSyncByte(uint8_t b);
bool isBlueGoalSyncByte(uint8_t b);
void decodeVisionPacket(const uint8_t *p);
void processVisionPackets();
void getLatestBallData(BallPacket &out);
void getLatestYellowGoalData(GoalPacket &out);
void getLatestBlueGoalData(GoalPacket &out);
