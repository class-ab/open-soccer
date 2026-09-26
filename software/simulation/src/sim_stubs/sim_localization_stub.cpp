#include "subsystems/localization.h"

#include <algorithm>
#include <mutex>

#include "sim_hal/sim_robot_io.h"

namespace {
std::mutex localizationMutex;
RobotPose robotPoses[2] = {
    {false, 0.0f, 0.0f, 0.0f, 0.0f, 0},
    {false, 0.0f, 0.0f, 0.0f, 0.0f, 0}};
FieldBall fieldBalls[2] = {
    {false, 0.0f, 0.0f, 0.0f, 0.0f, 0},
    {false, 0.0f, 0.0f, 0.0f, 0.0f, 0}};
OpponentRobot opponentsArr[2][3] = {};
int opponentCounts[2] = {0, 0};
}

void initLocalization() {}

void updateLocalization() {}

void getRobotPose(RobotPose &out) {
  std::lock_guard<std::mutex> lock(localizationMutex);
  out = robotPoses[g_simRobotSlot];
}

void getFieldBall(FieldBall &out) {
  std::lock_guard<std::mutex> lock(localizationMutex);
  out = fieldBalls[g_simRobotSlot];
}

void getOpponents(OpponentRobot *out, int maxOpponents, int &count) {
  std::lock_guard<std::mutex> lock(localizationMutex);
  count = 0;
  if (out == nullptr || maxOpponents <= 0) return;
  count = std::min(opponentCounts[g_simRobotSlot], maxOpponents);
  for (int i = 0; i < count; ++i) out[i] = opponentsArr[g_simRobotSlot][i];
}

void sim_set_localization(int slot, const RobotPose &pose, const FieldBall &ball,
                          const OpponentRobot *newOpponents, int count) {
  std::lock_guard<std::mutex> lock(localizationMutex);
  robotPoses[slot] = pose;
  fieldBalls[slot] = ball;
  opponentCounts[slot] = newOpponents == nullptr ? 0 : std::min(count, 3);
  for (int i = 0; i < opponentCounts[slot]; ++i) opponentsArr[slot][i] = newOpponents[i];
}

void sim_get_localization(int slot, RobotPose &pose, FieldBall &ball) {
  std::lock_guard<std::mutex> lock(localizationMutex);
  pose = robotPoses[slot];
  ball = fieldBalls[slot];
}
