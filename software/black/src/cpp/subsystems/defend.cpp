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

// Drive home to a point in front of our own goal, rotating to face the ball.
// Returns true if a movement was commanded, false otherwise (e.g. localisation
// unavailable or no fresh ball data).
bool returnHomeAndFaceBall(const localisation::Frame &frame) {
  BallPacket ball;
  getLatestBallData(ball);

  if (!frame.valid) {
    return false;
  }

  // Defensive point: DEFEND_DIST_FROM_OWN_GOAL_CM in front of our own goal. In
  // the localisation field frame our own goal is 180 deg away from "forwards"
  // (forwards points at the opponent goal), at a distance of half the goal
  // separation from the centre.
  float distFromCentre = frame.goalDistanceCm / 2.0f - DEFEND_DIST_FROM_OWN_GOAL_CM;
  if (distFromCentre < 0.0f) distFromCentre = 0.0f;
  float targetAngleDeg = 180.0f; // toward our own goal

  // Use the localisation profile only for translation (direction + speed).
  // The heading/rotation passed here is ignored -- we steer orientation
  // ourselves so the robot constantly points at the ball instead.
  MoveProfile prof;
  if (!localisation::computeTargetMoveProfile(frame, targetAngleDeg, distFromCentre,
                                              0.0f, prof)) {
    return false;
  }

  // Continuously point at the ball using the same smooth PID as chasing. If the
  // ball is out of sight, face "forwards" (toward the play) instead.
  float effectiveYaw = YAW_SIGN * currentYawDeg;
  bool ballFresh = ball.detected &&
                   (millis() - lastBallPacketMs) <= BALL_DATA_TIMEOUT_MS;
  if (ballFresh) {
    desiredHeadingDeg = effectiveYaw + ball.angleDeg;
  } else {
    desiredHeadingDeg = effectiveYaw + forwardsBearing(frame);
  }
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
