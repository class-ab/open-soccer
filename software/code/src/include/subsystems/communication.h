#pragma once

#include <stdint.h>

struct RobotPose;
struct FieldBall;
struct OpponentRobot;

// ============================================================
// Communication Subsystem Interface
// ============================================================
// Robot-to-robot wireless communication using nRF24L01+
// Allows each robot to access the other robot's pose, ball, and opponents
// Button 2 selects which robot this device is (1 or 2)
//   - Not pressed: Robot 1
//   - Pressed: Robot 2

void initCommunication();
void updateCommunication();

// Get the other robot's pose/ball/opponents
// These mirror the localization.h interface but for the remote robot
void getRemoteRobotPose(RobotPose &out);
void getRemoteFieldBall(FieldBall &out);
void getRemoteOpponents(OpponentRobot *out, int maxOpponents, int &count);

// Get the currently selected robot number (1 or 2) via button1
uint8_t getCurrentRobotNumber();

// Diagnostics
void printCommunicationStats();
