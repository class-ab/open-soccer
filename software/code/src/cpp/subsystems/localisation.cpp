#include "include/subsystems/localisation.h"

#include "include/subsystems/robot_config.h"

#include <cmath>

namespace localisation {

namespace {
// Named LOCAL_PI to avoid the Arduino core's `PI` macro.
constexpr float LOCAL_PI = 3.14159265358979323846f;

// Unit vector 90 deg counter-clockwise of a unit vector (robot frame).
Vec2 perpLeft(const Vec2 &v) {
  return {-v.y, v.x};
}

float rawWrapDeg(float a) {
  while (a > 180.0f) a -= 360.0f;
  while (a < -180.0f) a += 360.0f;
  return a;
}
} // namespace

float wrapDeg(float a) {
  return rawWrapDeg(a);
}

Vec2 polarToXY(float angleDeg, float distance) {
  float rad = angleDeg * LOCAL_PI / 180.0f;
  return {distance * std::cos(rad), distance * std::sin(rad)};
}

void xyToPolar(const Vec2 &v, float &angleDeg, float &distanceCM) {
  distanceCM = std::sqrt(v.x * v.x + v.y * v.y);
  angleDeg = std::atan2(v.y, v.x) * 180.0f / LOCAL_PI;
}

bool computeFrame(const GoalPacket &yellow, const GoalPacket &blue, Frame &out) {
  out.valid = false;

  if (!yellow.detected || !blue.detected) {
    return false;
  }

  // Robot-relative vectors to the two goals: CB (yellow) and CD (blue).
  Vec2 cb = polarToXY(yellow.angleDeg, yellow.distanceCM);
  Vec2 cd = polarToXY(blue.angleDeg, blue.distanceCM);

  // Centre point: CA = (CB + CD) / 2.
  Vec2 centre = {(cb.x + cd.x) * 0.5f, (cb.y + cd.y) * 0.5f};

  // Front direction of the field: AB = CB - CA (toward the yellow goal).
  Vec2 ab = {cb.x - centre.x, cb.y - centre.y};
  float abLen = std::sqrt(ab.x * ab.x + ab.y * ab.y);

  // Distance between the goals (BD) -- kept for reference / scaling.
  Vec2 bd = {cd.x - cb.x, cd.y - cb.y};
  float goalDist = std::sqrt(bd.x * bd.x + bd.y * bd.y);

  out.centre = centre;
  out.goalDistanceCm = goalDist;

  if (abLen > 1e-3f) {
    out.frontDir = {ab.x / abLen, ab.y / abLen};
  } else {
    // Goals coincide in direction; fall back to pointing straight ahead.
    out.frontDir = {1.0f, 0.0f};
  }
  out.frontLeft = perpLeft(out.frontDir);
  out.frontAngleDeg = std::atan2(ab.y, ab.x) * 180.0f / LOCAL_PI;

  out.valid = true;
  return true;
}

bool computeTargetMoveProfile(const Frame &f,
                              float targetPointAngleDeg,
                              float targetPointDistCm,
                              float targetHeadingDeg,
                              MoveProfile &out) {
  if (!f.valid) {
    return false;
  }

  // ----- Field "forwards" (the 0 angle) tied to our alliance -----
  // f.frontAngleDeg is the robot-relative bearing to the YELLOW goal. On the
  // blue alliance "forwards" is the yellow goal itself; on the yellow
  // alliance it is the opposite direction (180 deg away, toward the blue goal).
  float forwardsBearingDeg =
      isYellowAlliance ? rawWrapDeg(f.frontAngleDeg + 180.0f) : f.frontAngleDeg;

  // The robot's current heading in the field frame. Facing "forwards" means
  // field heading 0; turning CCW increases the field heading, and the bearing
  // to "forwards" from the robot front runs the opposite way.
  float robotHeadingInFieldDeg = rawWrapDeg(-forwardsBearingDeg);

  // ----- Movement direction: drive directly at the target point -----
  // Unit axes of the field frame, resolved in the robot-relative frame.
  Vec2 eF = polarToXY(forwardsBearingDeg, 1.0f); // unit "forwards"
  Vec2 eL = perpLeft(eF);                         // unit left of "forwards"

  // Target point from the field centre A, resolved in robot-relative coords.
  float rad = targetPointAngleDeg * LOCAL_PI / 180.0f;
  float c = std::cos(rad), s = std::sin(rad);
  Vec2 at = {
    targetPointDistCm * (c * eF.x + s * eL.x),
    targetPointDistCm * (c * eF.y + s * eL.y),
  };

  // Robot -> target = centre(robot->A) + A->target.
  Vec2 toTarget = {f.centre.x + at.x, f.centre.y + at.y};
  float bearingDeg, distCm;
  xyToPolar(toTarget, bearingDeg, distCm);

  // ----- Translation speed: proportional to remaining distance -----
  float speed;
  float remaining = distCm - LOC_TARGET_REACH_DIST_CM;
  if (remaining <= 0.0f) {
    speed = 0.0f;
  } else {
    float t = constrain(remaining / LOC_RAMP_RANGE_CM, 0.0f, 1.0f);
    speed = LOC_MIN_SPEED + t * (LOC_MAX_SPEED - LOC_MIN_SPEED);
  }

  // ----- Rotation: correct heading toward the target heading -----
  // Positive rotation is CCW (matches the simulator). A heading error that
  // requires turning CCW (target > current field heading) yields positive
  // rotation. LOC_ROT_EFFORT_DEG is the error that saturates rotation.
  float headingErr = rawWrapDeg(targetHeadingDeg - robotHeadingInFieldDeg);
  float rotation = constrain(headingErr / LOC_ROT_EFFORT_DEG, -1.0f, 1.0f);

  out.active = true;
  out.movementDirectionDeg = bearingDeg;
  out.speed = speed;
  out.rotationSpeed = rotation;
  out.lastUpdateMs = millis();
  return true;
}

} // namespace localisation
