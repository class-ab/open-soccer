#include "subsystems/communication.h"

#include "subsystems/localization.h"
#include "subsystems/strategy.h"
#include "sim_hal/sim_robot_io.h"

namespace {
thread_local uint8_t currentRobotNumber = 1;
}

bool initCommunication() { return true; }

void updateCommunication() {}

void getRemoteRobotPose(RobotPose &out) {
  RobotPose pose;
  FieldBall ball;
  sim_get_localization(1 - g_simRobotSlot, pose, ball);
  out = pose;
}

void getRemoteRobotState(LocalState &out) {
  sim_get_published_local_state(1 - g_simRobotSlot, out);
}

void getRemoteFieldBall(FieldBall &out) {
  RobotPose pose;
  FieldBall ball;
  sim_get_localization(1 - g_simRobotSlot, pose, ball);
  out = ball;
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
