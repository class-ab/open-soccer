#include "subsystems/localization.h"

#include <algorithm>
#include <mutex>

#include "sim_hal/sim_robot_io.h"

namespace {
std::mutex localizationMutex;
RobotPose robotPose = {false, 0.0f, 0.0f, 0.0f, 0.0f, 0};
FieldBall fieldBall = {false, 0.0f, 0.0f, 0.0f, 0.0f, 0};
OpponentRobot opponents[3] = {};
int opponentCount = 0;
}

void initLocalization() {}

void updateLocalization() {}

void getRobotPose(RobotPose &out) {
  std::lock_guard<std::mutex> lock(localizationMutex);
  out = robotPose;
}

void getFieldBall(FieldBall &out) {
  std::lock_guard<std::mutex> lock(localizationMutex);
  out = fieldBall;
}

void getOpponents(OpponentRobot *out, int maxOpponents, int &count) {
  std::lock_guard<std::mutex> lock(localizationMutex);
  count = 0;
  if (out == nullptr || maxOpponents <= 0) return;
  count = std::min(opponentCount, maxOpponents);
  for (int i = 0; i < count; ++i) out[i] = opponents[i];
}

void sim_set_localization(const RobotPose &pose, const FieldBall &ball,
                          const OpponentRobot *newOpponents, int count) {
  std::lock_guard<std::mutex> lock(localizationMutex);
  robotPose = pose;
  fieldBall = ball;
  opponentCount = newOpponents == nullptr ? 0 : std::min(count, 3);
  for (int i = 0; i < opponentCount; ++i) opponents[i] = newOpponents[i];
}