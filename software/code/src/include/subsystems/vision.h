#pragma once

#include "include/subsystems/robot_state.h"

void initBallTracking();
bool isBallSyncByte(uint8_t b);
void decodeBallPacket(const uint8_t *p);
void processBallPacket();
void getLatestBallData(BallPacket &out);
void chaseTick();
