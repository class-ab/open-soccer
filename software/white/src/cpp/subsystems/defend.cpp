#include "include/subsystems/defend.h"

#include <Arduino.h>

#include "include/subsystems/robot_config.h"
#include "include/subsystems/robot_state.h"
#include "include/subsystems/vision.h"
#include "include/subsystems/drivebase.h"
#include "include/subsystems/imu.h"

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

// Position on the goal-to-ball line at DEFEND_DIST from own goal, facing the
// ball. Falls back to return-home if ball data is unavailable.
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

  // Own goal centre G in robot-relative coords.
  localisation::Vec2 goalCentre = localisation::polarToXY(
      frame.frontAngleDeg, frame.goalDistanceCm / 2.0f);
  goalCentre.x = frame.centre.x - goalCentre.x;
  goalCentre.y = frame.centre.y - goalCentre.y;

  // Ball and goal centre in the field frame (centred on A).
  localisation::Vec2 ballField = {
      localisation::polarToXY(ball.angleDeg, ball.distanceCM).x - frame.centre.x,
      localisation::polarToXY(ball.angleDeg, ball.distanceCM).y - frame.centre.y,
  };
  localisation::Vec2 goalField = {
      goalCentre.x - frame.centre.x,
      goalCentre.y - frame.centre.y,
  };

  // Direction from own goal to the ball in the field frame.
  localisation::Vec2 gb = {
      ballField.x - goalField.x,
      ballField.y - goalField.y,
  };

  // Target y: at DEFEND_DIST from the goal line.
  float targetY = goalField.y + DEFEND_DIST_FROM_OWN_GOAL_CM;

  // Intersect the goal-to-ball line with that y.
  float t = 0.0f;
  if (fabsf(gb.y) > 1e-3f) {
    t = (targetY - goalField.y) / gb.y;
  }

  // Clamp so the robot never goes more than DEFEND_MAX_FORWARD_CM past the
  // defend line (toward the ball).
  float tMax = DEFEND_MAX_FORWARD_CM / fabsf(gb.y);
  t = constrain(t, -1000.0f, tMax);

  // Target in the field frame, then convert to robot-relative.
  localisation::Vec2 targetField = {
      goalField.x + gb.x * t,
      goalField.y + gb.y * t,
  };
  localisation::Vec2 targetRobot = {
      targetField.x + frame.centre.x,
      targetField.y + frame.centre.y,
  };

  float targetAngleDeg, targetDistCm;
  localisation::xyToPolar(targetRobot, targetAngleDeg, targetDistCm);

  MoveProfile prof;
  if (!localisation::computeTargetMoveProfile(frame, targetAngleDeg, targetDistCm,
                                              0.0f, prof)) {
    return false;
  }

  // Face the ball. Use the ball's bearing from field centre A so the heading
  // stays consistent as the robot strafes along the defend line.
  float ballAngleDeg, ballDistCm;
  localisation::xyToPolar(ballField, ballAngleDeg, ballDistCm);
  desiredHeadingDeg = effectiveYaw + ballAngleDeg - forwardsBearing(frame);
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
