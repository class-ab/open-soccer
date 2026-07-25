#ifndef SUBSYSTEMS_DRIVEBASE_H
#define SUBSYSTEMS_DRIVEBASE_H

#include <Arduino.h>

// Set to +1.0 or -1.0 depending on which way your IMU's yaw increases -
// see the tuning note above its definition in drivebase.cpp. Also read
// by ball_tracking's chaseTick() to convert the ball's chassis-relative
// bearing into a world-frame heading target, the same trick move() uses
// internally via startYaw.
extern const float YAW_SIGN;

// Heading-hold setpoint, degrees, in the same world frame as
// currentYawDeg (imu.h). move() ramps this toward each move's target
// heading at an accel-limited rate; chaseTick() (ball_tracking.cpp)
// instead points it straight at the ball every tick.
extern float desiredHeadingDeg;

// Configures the four motor output pins. Call once from setup().
void initDrivebase();

// Immediately zeroes all four motor outputs.
void stopAllMotors();

// Field-oriented holonomic drive. direction_deg is a FIXED, FIELD-relative
// direction (a world-frame compass angle, NOT relative to however the
// chassis currently happens to be pointed - see the long comment above
// drive()'s definition in drivebase.cpp). speed is 0.0-1.0. rotation is a
// -1.0..1.0 motor-scale spin command, typically headingCorrection()'s
// output.
void drive(
  float direction_deg,
  float speed,
  float rotation);

// Runs one pre-planned move: accel-limited translation toward
// direction_deg for duration_ms, plus an independently accel-limited,
// as-fast-as-possible rotation toward targetRotation_deg. Blocks for the
// duration of the move, calling systemTick() every tick so battery/
// display/ball-packet handling keep running throughout. See the long
// comment above its definition in drivebase.cpp for exactly how
// translation and rotation timing are decoupled.
void move(
  float direction_deg,
  unsigned long duration_ms,
  float targetRotation_deg,
  float maxSpeed);

// Clears heading-hold PID state (integral, last error, timer). Call at
// the start of any fresh heading-hold session - move() does this
// internally; chaseTick() deliberately does not, since it's meant to be
// called repeatedly across many ticks as one continuous session.
void resetHeadingPID();

// Runs one iteration of the heading-hold PID (desiredHeadingDeg vs
// currentYawDeg) and returns the rotation correction to feed into
// drive(). Used by both move() and chaseTick().
float headingCorrection();

#endif
