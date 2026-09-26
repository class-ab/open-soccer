#include "subsystems/communication.h"

#include "subsystems/localization.h"
#include "subsystems/strategy.h"

namespace {
uint8_t currentRobotNumber = 1;
}

bool initCommunication() { return true; }

void updateCommunication() {}

void getRemoteRobotPose(RobotPose &out) {
  out = {false, 0.0f, 0.0f, 0.0f, 0.0f, 0};
}

void getRemoteRobotState(LocalState &out) {
  out = {RobotState::damaged, RobotGoal::none};
}

void getRemoteFieldBall(FieldBall &out) {
  out = {false, 0.0f, 0.0f, 0.0f, 0.0f, 0};
}

void getRemoteOpponents(OpponentRobot *out, int maxOpponents, int &count) {
  count = 0;
  if (out == nullptr || maxOpponents <= 0) return;
  for (int i = 0; i < maxOpponents; ++i) {
    out[i] = {false, 0.0f, 0.0f, 0.0f, 0};
  }
}

uint8_t getCurrentRobotNumber() { return currentRobotNumber; }

void selectCurrentRobotNumber(uint8_t robotNumber) {
  if (robotNumber == 1 || robotNumber == 2) currentRobotNumber = robotNumber;
}

void printCommunicationStats() {}