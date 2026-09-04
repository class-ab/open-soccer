#include "include/subsystems/defend.h"

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

// Robot-relative bearing to the field "forwards" (the 0 angle): toward the
// opposite-coloured goal. Mirrors localisation's reference-frame definition.
float forwardsBearing(const localisation::Frame &f) {
  return isYellowAlliance
    ? toSigned(f.frontAngleDeg + 180.0f)
    : f.frontAngleDeg;
}

// Dot product of two robot-frame vectors.
float dot(const localisation::Vec2 &a, const localisation::Vec2 &b) {
  return a.x * b.x + a.y * b.y;
}

// Position the robot on the goal-to-ball line so it sits between the own goal
// and the ball, keeping it within DEFEND_DIST..DEFEND_DIST+MAX_FORWARD of the
// goal. Falls back to return-home if ball data is unavailable.
// Returns true if a movement was commanded.
bool returnHomeAndFaceBall(const localisation::Frame &frame) {
  BallPacket ball;
  getLatestBallData(ball);

  if (!frame.valid) {
    return false;
  }

  float effectiveYaw = YAW_SIGN * currentYawDeg;
  bool ballFresh = ball.detected &&
                   (millis() - lastBallPacketMs) <= BALL_DATA_TIMEOUT_MS;

  // If the ball is not visible, fall back to returning to the fixed home
  // position and facing forwards (toward the play).
  if (!ballFresh) {
    float distFromCentre = frame.goalDistanceCm / 2.0f - DEFEND_DIST_FROM_OWN_GOAL_CM;
    if (distFromCentre < 0.0f) distFromCentre = 0.0f;

    MoveProfile prof;
    if (!localisation::computeTargetMoveProfile(frame, 180.0f, distFromCentre,
                                                0.0f, prof)) {
      return false;
    }

    desiredHeadingDeg = effectiveYaw + forwardsBearing(frame);
    float rotation = -headingCorrection();
    drive(effectiveYaw - prof.movementDirectionDeg, prof.speed, rotation);
    return true;
  }

  // Field frame: origin at field centre A, x-axis = "forwards" (0 deg),
  // y-axis = left of forwards. Own goal is at (-d, 0) with
  // d = goalDistance/2.
  const float d = frame.goalDistanceCm / 2.0f;
  const localisation::Vec2 eF =
      localisation::polarToXY(forwardsBearing(frame), 1.0f);
  const localisation::Vec2 eL = {-eF.y, eF.x};

  // Ball in field-frame coords.
  const localisation::Vec2 toBallRobot =
      localisation::polarToXY(ball.angleDeg, ball.distanceCM);
  const localisation::Vec2 aToBallRobot = {
      toBallRobot.x - frame.centre.x,
      toBallRobot.y - frame.centre.y,
  };
  const localisation::Vec2 ballField = {
      dot(aToBallRobot, eF),
      dot(aToBallRobot, eL),
  };

  // Goal -> ball line in field coords.
  localisation::Vec2 gbField = {
      ballField.x + d,
      ballField.y,
  };
  const float gbLen = std::sqrt(gbField.x * gbField.x + gbField.y * gbField.y);

  // Ideal blocking distance from the goal: the defend line, but advance toward
  // the ball up to DEFEND_MAX_FORWARD_CM of extra forward travel.
  float targetFromGoal = DEFEND_DIST_FROM_OWN_GOAL_CM;
  if (gbLen > DEFEND_DIST_FROM_OWN_GOAL_CM) {
    targetFromGoal = std::min(gbLen, DEFEND_DIST_FROM_OWN_GOAL_CM + DEFEND_MAX_FORWARD_CM);
  }

  // Target on the goal-ball line at targetFromGoal from the goal.
  const float ux = (gbLen > 1e-3f) ? gbField.x / gbLen : 0.0f;
  const float uy = (gbLen > 1e-3f) ? gbField.y / gbLen : 0.0f;
  const localisation::Vec2 targetField = {
      -d + ux * targetFromGoal,
      0.0f + uy * targetFromGoal,
  };

  // Target in field-frame polar (bearing from A, distance from A).
  const float targetAngleDeg = std::atan2(targetField.y, targetField.x) * 180.0f / PI;
  const float targetDistCm =
      std::sqrt(targetField.x * targetField.x + targetField.y * targetField.y);

  MoveProfile prof;
  if (!localisation::computeTargetMoveProfile(frame, targetAngleDeg, targetDistCm,
                                              0.0f, prof)) {
    return false;
  }

  // Face the ball.
  desiredHeadingDeg = effectiveYaw + ball.angleDeg;
  float rotation = -headingCorrection();

  drive(effectiveYaw - prof.movementDirectionDeg, prof.speed, rotation);
  return true;
}

} // namespace

void initDefend() {
  // Nothing to initialise yet; motors/dribbler are managed by the loop.
  Serial.println("Defend behavior ready");
}

void defendTick() {
  // Only drive it when the robot is actually enabled. If not, stop.
  if (!robotCurrentlyRunning) {
    dribblerShouldRun = false;
    stopAllDriveMotors();
    return;
  }

  // Try to navigate home; if localisation/ball data is unavailable, stop.
  localisation::Frame frame;
  bool haveFrame = localisation::computeFrame(latestYellowGoalPacket,
                                              latestBlueGoalPacket, frame);
  if (!haveFrame || !returnHomeAndFaceBall(frame)) {
    dribblerShouldRun = false;
    stopAllDriveMotors();
    return;
  }

  // Defending does not need the dribbler.
  dribblerShouldRun = false;
}
