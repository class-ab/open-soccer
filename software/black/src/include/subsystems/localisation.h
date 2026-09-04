#pragma once

#include "include/subsystems/robot_state.h"

/*
  Camera localisation based on the two goal vectors (see
  https://bozotics.github.io/open/strat/localisation/).

  The robot measures a vector to the yellow goal (CB) and to the blue goal
  (CD) in polar form relative to its front (the GoalPacket angle/distance
  fields). Rather than trust either absolute distance, we:

    1. Sum the two vectors -> CE = CB + CD, then halve -> CA = CE/2. CA is the
       robot-relative vector to the centre point A of the goal line. Averaging
       the two (noisy) distances makes this reference more stable than either
       alone.

    2. Build AB = CB - CA. This is the direction from the field centre toward
       the yellow goal. From it we derive the field "forwards" axis (below).

  We then aim at a target *without* ever trying to always face one goal.

  Reference frame / "forwards" (the field 0 angle)
  ------------------------------------------------
  The field polar frame is anchored at the goal-line centre A, with the 0
  angle pointing toward the OPPOSITE-coloured goal from our alliance. So when
  the robot is on the YELLOW alliance, 0 = toward the blue goal; when on the
  BLUE alliance, 0 = toward the yellow goal. Angles increase CCW (positive =
  to the left of "forwards"). This "forwards" is read from the shared global
  isYellowAlliance, so it automatically matches the robot's current alliance.

  Polar targets are given in this field frame, so a desired destination or
  heading is expressed relative to "forwards" rather than as an absolute
  heading.

  Vector convention (matching GoalPacket / vision data):
    - robot-relative Cartesian frame: +x = robot front/dribbler, +y = robot's
      left (CCW-positive when viewed from above).
    - polar angle measured from +x, positive toward +y.
*/

namespace localisation {

// Robot-relative vector in Cartesian form (units = cm).
struct Vec2 {
  float x;
  float y;
};

// Triangulated robot-relative field frame anchored at the goal-line centre.
struct Frame {
  bool   valid;          // false if either goal was not detected
  Vec2   centre;         // CA: robot -> centre point A (cm)
  Vec2   frontDir;       // unit vector from A toward the yellow goal
  Vec2   frontLeft;      // unit vector 90 deg CCW of frontDir
  float  frontAngleDeg;  // robot-relative bearing to the yellow goal (deg)
  float  goalDistanceCm; // |BD| : (noisy) distance between the two goals
};

// Triangulate the yellow/blue goal vectors. Returns false (leaving out
// invalid) if either goal is not detected.
bool computeFrame(const GoalPacket &yellow, const GoalPacket &blue, Frame &out);

// Compute a MoveProfile to drive holonomically toward a target while rotating
// to finish facing a target heading.
//   f                  : the triangulated frame
//   targetPointAngleDeg: bearing of the target from the field centre A, in the
//                        field frame (0 = "forwards"), CCW-positive
//   targetPointDistCm  : distance of the target from the field centre (cm)
//   targetHeadingDeg   : heading the robot should finish facing, in the same
//                        field frame (0 = "forwards"), CCW-positive
// Fills `out` with:
//   movementDirectionDeg : robot-relative bearing to travel directly at target
//   speed                : proportional to remaining distance (config-clamped)
//   rotationSpeed        : normalised heading correction (positive = CCW)
//   active/lastUpdateMs  : set
// Returns false if the frame is invalid.
bool computeTargetMoveProfile(const Frame &f,
                              float targetPointAngleDeg,
                              float targetPointDistCm,
                              float targetHeadingDeg,
                              MoveProfile &out);

// Helpers (robot-relative frame, angle measured from +x toward +y).
Vec2 polarToXY(float angleDeg, float distance);
void xyToPolar(const Vec2 &v, float &angleDeg, float &distanceCM);
float wrapDeg(float a); // wrap to [-180, 180]

} // namespace localisation
