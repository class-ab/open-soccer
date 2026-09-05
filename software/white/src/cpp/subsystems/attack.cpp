#include "include/subsystems/attack.h"

#include <Arduino.h>

#include "include/subsystems/robot_config.h"
#include "include/subsystems/robot_state.h"
#include "include/subsystems/vision.h"
#include "include/subsystems/drivebase.h"
#include "include/subsystems/imu.h"

#include <cmath>

namespace {

// Wrap an angle to [0, 360).
float to0To360(float deg) {
  while (deg < 0.0f) deg += 360.0f;
  while (deg >= 360.0f) deg -= 360.0f;
  return deg;
}

// Signed angle in [-180, 180].
float toSigned(float deg) {
  deg = to0To360(deg);
  if (deg > 180.0f) deg -= 360.0f;
  return deg;
}

// Front-scoring offset angle O_G: O_G = G capped to +/-90 (article, M_G=1 form).
float offsetGoal(float G) {
  if (G <= 180.0f) {
    return std::min(G, 90.0f);
  }
  return std::max(G - 360.0f, -90.0f);
}

// Speed proportional to remaining distance (ball or goal chase ramp).
float rampSpeed(float remainingCm, float minS, float maxS, float rampRange) {
  if (remainingCm <= 0.0f) return 0.0f;
  float t = constrain(remainingCm / rampRange, 0.0f, 1.0f);
  return minS + t * (maxS - minS);
}

// Distance is assumed fresh; returns the opponent (opposite-colour) goal
// packet, or updates `lastGoalMs` with the matching timestamp.
bool getOpponentGoal(GoalPacket &out, unsigned long &lastMs) {
  if (isYellowAlliance) {
    out = latestBlueGoalPacket;
    lastMs = lastBlueGoalPacketMs;
  } else {
    out = latestYellowGoalPacket;
    lastMs = lastYellowGoalPacketMs;
  }
  return out.detected && (millis() - lastMs) <= BALL_DATA_TIMEOUT_MS;
}

} // namespace

void initAttack() {
  // Nothing to initialise yet; dribbler/motors are managed by the loop.
  Serial.println("Attack behavior ready");
}

void attackTick() {
  // Attacking: keep the dribbler running in both phases (it helps collect and
  // hold the ball).
  dribblerShouldRun = true;

  if (!hasBall) {
    // ---- Phase 1: chase the ball (straight for it, no orbit) ----
    BallPacket ball;
    getLatestBallData(ball);

    if (!ball.detected || (millis() - lastBallPacketMs) > BALL_DATA_TIMEOUT_MS) {
      stopAllDriveMotors();
      return;
    }

    float effectiveYaw = YAW_SIGN * currentYawDeg;

    // Drive directly at the ball bearing.
    float moveDeg = ball.angleDeg;

    // Face the ball so the dribbler captures it.
    desiredHeadingDeg = effectiveYaw + ball.angleDeg;
    float rotation = -headingCorrection();

    float speed = rampSpeed(ball.distanceCM - BALL_TARGET_DISTANCE_CM,
                            CHASE_MIN_SPEED, CHASE_MAX_SPEED, CHASE_RAMP_RANGE_CM);

    // drive() expects a world/effectiveYaw-relative direction; subtract the
    // desired robot-relative bearing to reuse the established convention.
    drive(effectiveYaw - moveDeg, speed, rotation);
  } else {
    // ---- Phase 2: score (keep the orbiting from the article) ----
    GoalPacket goal;
    unsigned long goalMs;
    if (!getOpponentGoal(goal, goalMs)) {
      // Can't see the goal to aim at; hold the ball and stop.
      stopAllDriveMotors();
      return;
    }

    float goal0 = to0To360(goal.angleDeg);
    float goalDeg = toSigned(goal0);
    float goalDist = goal.distanceCM;

    float effectiveYaw = YAW_SIGN * currentYawDeg;

    // Rotate to face the goal so the frontal dribbler pushes the ball in.
    desiredHeadingDeg = effectiveYaw + goalDeg;
    float rotation = -headingCorrection();

    float direction;
    float speed;
    if (goalDist <= PUSH_DIST_CM) {
      // Close enough: drive straight into the goal with the ball.
      direction = effectiveYaw - goalDeg;
      speed = PUSH_SPEED;
    } else {
      // Orbit around to face the goal first (article's front scoring offset).
      float goalOff = offsetGoal(goal0);
      float scoreMove0 = to0To360(goal0 + goalOff * GOAL_OFFSET_MULT);
      float scoreMove = toSigned(scoreMove0);
      direction = effectiveYaw - scoreMove;
      speed = rampSpeed(goalDist - PUSH_DIST_CM,
                        SCORE_MIN_SPEED, SCORE_MAX_SPEED, SCORE_RAMP_RANGE_CM);
    }

    drive(direction, speed, rotation);
  }
}

// void attackTick() {
//   GoalPacket goal;
//   unsigned long goalMs;
//   if (!getOpponentGoal(goal, goalMs)) {
//     // Can't see the goal to aim at; hold the ball and stop.
//     stopAllDriveMotors();
//     return;
//   }

//   float goal0 = to0To360(goal.angleDeg);
//   float goalDeg = toSigned(goal0);
//   float goalDist = goal.distanceCM;

//   float effectiveYaw = YAW_SIGN * currentYawDeg;

//   // Rotate to face the goal so the frontal dribbler pushes the ball in.
//   desiredHeadingDeg = effectiveYaw + goalDeg;
//   float rotation = -headingCorrection();

//   float direction;
//   float speed;
//   direction = effectiveYaw - goalDeg;
//   speed = PUSH_SPEED;

//   drive(direction, speed, rotation);
// }