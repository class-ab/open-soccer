#pragma once

#include <stdint.h>

struct RobotPose;
struct FieldBall;
struct OpponentRobot;
struct LocalState;

// ============================================================
// Communication Subsystem Interface
// ============================================================
// Robot-to-robot wireless communication using nRF24L01+
// Allows each robot to access the other robot's pose, ball, and opponents
// Select the identity used for radio addressing. Robot 1 is the attacker and
// robot 2 is the defender; strategy receives the same selection.

// Returns false when the nRF24L01+ cannot be reached over SPI.
bool initCommunication();
void updateCommunication();

// Get the other robot's pose/ball/opponents
// These mirror the localization.h interface but for the remote robot
void getRemoteRobotPose(RobotPose &out);
void getRemoteRobotState(LocalState &out);
void getRemoteFieldBall(FieldBall &out);
void getRemoteOpponents(OpponentRobot *out, int maxOpponents, int &count);

// Get or select the currently selected robot number (1 or 2).
uint8_t getCurrentRobotNumber();
void selectCurrentRobotNumber(uint8_t robotNumber);

// Diagnostics
void printCommunicationStats();
